// Adapted from the previous FrozenCostmap replay: installed plugins, no costmap layers.
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_loader.hpp>
#include <rcutils/logging.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

using Json = nlohmann::json;

class FrozenCostmap : public nav2_costmap_2d::Costmap2DROS {
public:
  explicit FrozenCostmap(const Json & c) : Costmap2DROS("frozen_global_costmap") {
    global_frame_ = c.at("frame").get<std::string>();
    rolling_window_ = false;
    track_unknown_space_ = c.at("track_unknown_space");
    footprint_padding_ = c.at("footprint_padding");
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      global_frame_, false, track_unknown_space_);
    layered_costmap_->resizeMap(c.at("size_x"), c.at("size_y"), c.at("resolution"),
      c.at("origin").at("position").at("x"), c.at("origin").at("position").at("y"));
    std::vector<geometry_msgs::msg::Point> points;
    if (!nav2_costmap_2d::makeFootprintFromString(c.at("footprint"), points)) {
      throw std::runtime_error("footprint parse failed");
    }
    setRobotFootprint(points);
  }
};

geometry_msgs::msg::PoseStamped readPose(const Json & j) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = j.at("header").at("frame_id");
  p.header.stamp.sec = j.at("header").at("stamp").at("sec");
  p.header.stamp.nanosec = j.at("header").at("stamp").at("nanosec");
  const auto & pos = j.at("pose").at("position");
  const auto & q = j.at("pose").at("orientation");
  p.pose.position.x = pos.at("x"); p.pose.position.y = pos.at("y"); p.pose.position.z = pos.at("z");
  p.pose.orientation.x = q.at("x"); p.pose.orientation.y = q.at("y");
  p.pose.orientation.z = q.at("z"); p.pose.orientation.w = q.at("w");
  return p;
}

Json writePose(const geometry_msgs::msg::PoseStamped & p) {
  return {{"header", {{"frame_id", p.header.frame_id}, {"stamp", {
    {"sec", p.header.stamp.sec}, {"nanosec", p.header.stamp.nanosec}}}}},
    {"pose", {{"position", {{"x", p.pose.position.x}, {"y", p.pose.position.y}, {"z", p.pose.position.z}}},
    {"orientation", {{"x", p.pose.orientation.x}, {"y", p.pose.orientation.y},
    {"z", p.pose.orientation.z}, {"w", p.pose.orientation.w}}}}}};
}

void writePath(const std::string & prefix, const nav_msgs::msg::Path & path) {
  std::ofstream csv(prefix+".csv");
  csv << "x_map,y_map,z_map,qx,qy,qz,qw\n" << std::setprecision(17);
  Json poses = Json::array();
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose;
    csv << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
      << p.orientation.x << ',' << p.orientation.y << ',' << p.orientation.z << ',' << p.orientation.w << '\n';
    poses.push_back(writePose(pose));
  }
  Json message = {{"header", {{"frame_id", path.header.frame_id}, {"stamp", {
    {"sec", path.header.stamp.sec}, {"nanosec", path.header.stamp.nanosec}}}}}, {"poses", poses}};
  std::ofstream(prefix+".json") << message.dump(2) << '\n';
}

void snapshot(const std::string & path, nav2_costmap_2d::Costmap2D * cm) {
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<char *>(cm->getCharMap()), cm->getSizeInCellsX()*cm->getSizeInCellsY());
  if (!f) {throw std::runtime_error("snapshot write failed");}
}

int main(int argc, char ** argv) {
  if (argc != 6) {return 2;}
  const std::string prefix = argv[4];
  Json result = {{"plugin_load_success", false}, {"configure_success", false},
    {"create_plan_success", false}, {"create_plan_returned", false}, {"exception", nullptr},
    {"returned_pose_count", 0}, {"planning_wall_ms", nullptr}, {"plugin_type", argv[5]},
    {"action_status", nullptr}, {"lifecycle_cleanup_success", false}};
  int rc = 0;
  try {
    Json input; std::ifstream(argv[1]) >> input;
    const auto & c = input.at("common"); const auto & request = input.at("request");
    const std::string id = request.at("planner_id");
    const auto start = readPose(request.at("start")), goal = readPose(request.at("goal"));
    if (!request.at("use_start").get<bool>()) {throw std::runtime_error("use_start must be true");}
    std::vector<unsigned char> raw(c.at("size_x").get<size_t>()*c.at("size_y").get<size_t>());
    std::ifstream f(argv[2], std::ios::binary);
    f.read(reinterpret_cast<char *>(raw.data()), raw.size());
    if (static_cast<size_t>(f.gcount()) != raw.size() || f.peek() != EOF) {
      throw std::runtime_error("raw size mismatch");
    }
    rclcpp::init(0, nullptr);
    // NodeOptions log-level does not configure the process-wide installed NavFn logger.
    if (rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK ||
      rcutils_logging_set_logger_level("planner_server", RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
      throw std::runtime_error("failed to enable installed-library DEBUG logger");
    }
    {
      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server", "",
        rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", argv[3], "--log-level", "debug"})
        .automatically_declare_parameters_from_overrides(true).use_intra_process_comms(true));
      nav_msgs::msg::Path::ConstSharedPtr unsmoothed;
      size_t raw_messages = 0;
      rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_sub;
      if (id == "Smac2D") {
        raw_sub = node->create_subscription<nav_msgs::msg::Path>("unsmoothed_plan", 1,
          [&](nav_msgs::msg::Path::ConstSharedPtr message) {unsmoothed = std::make_shared<nav_msgs::msg::Path>(*message); ++raw_messages;});
      }
      auto cm = std::make_shared<FrozenCostmap>(c);
      auto * master = cm->getCostmap();
      std::copy(raw.begin(), raw.end(), master->getCharMap());
      snapshot(prefix+".costmap-loaded.u8", master);
      pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
      auto plugin = loader.createSharedInstance(argv[5]);
      result["plugin_load_success"] = true;
      std::cout << "PLUGIN_LOAD_SUCCESS " << argv[5] << std::endl;
      plugin->configure(node, id, nullptr, cm); plugin->activate();
      result["configure_success"] = true;
      std::ofstream params(prefix+".parameters.txt");
      Json exact_params = Json::object();
      for (const auto & key : node->list_parameters({}, 100).names) {
        const auto p = node->get_parameter(key);
        params << key << ": " << p.value_to_string() << "\n";
        if (key.rfind(id+".", 0) == 0) {
          switch (p.get_type()) {
            case rclcpp::ParameterType::PARAMETER_BOOL: exact_params[key] = p.as_bool(); break;
            case rclcpp::ParameterType::PARAMETER_INTEGER: exact_params[key] = p.as_int(); break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE: exact_params[key] = p.as_double(); break;
            case rclcpp::ParameterType::PARAMETER_STRING: exact_params[key] = p.as_string(); break;
            default: throw std::runtime_error("unsupported plugin parameter type: "+key);
          }
        }
      }
      result["plugin_parameters"] = exact_params;
      result["footprint_padded_xy"] = Json::array();
      for (const auto & p : cm->getRobotFootprint()) {result["footprint_padded_xy"].push_back({p.x, p.y});}
      result["applied_request"] = {{"planner_id", id}, {"use_start", true},
        {"start", writePose(start)}, {"goal", writePose(goal)}};
      result["applied_common"] = c;
      result["applied_common"]["size_x"] = master->getSizeInCellsX();
      result["applied_common"]["size_y"] = master->getSizeInCellsY();
      result["applied_common"]["resolution"] = master->getResolution();
      result["applied_common"]["origin"]["position"]["x"] = master->getOriginX();
      result["applied_common"]["origin"]["position"]["y"] = master->getOriginY();
      result["applied_common"]["frame"] = cm->getGlobalFrameID();
      std::ofstream(prefix+".configured.json") << result.dump(2) << '\n';
      unsigned int sx, sy, gx, gy;
      if (!master->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !master->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        throw std::runtime_error("start/goal outside frozen master");
      }
      result["floor_start_cell"] = {sx, sy}; result["floor_goal_cell"] = {gx, gy};
      snapshot(prefix+".costmap-before-plan.u8", master);
      nav_msgs::msg::Path path;
      const auto begin = std::chrono::steady_clock::now();
      try {
        path = plugin->createPlan(start, goal);
        result["create_plan_returned"] = true;
        result["create_plan_success"] = !path.poses.empty();
      } catch (const std::exception & e) {
        result["exception"] = e.what();
        std::cerr << "CREATE_PLAN_EXCEPTION: " << e.what() << std::endl;
      }
      result["planning_wall_ms"] = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-begin).count();
      result["returned_pose_count"] = path.poses.size();
      result["path_header"] = {{"frame_id", path.header.frame_id}, {"stamp", {
        {"sec", path.header.stamp.sec}, {"nanosec", path.header.stamp.nanosec}}}};
      writePath(prefix+".path", path);
      // Process only the debug message after createPlan; never alter search/smoother parameters.
      rclcpp::spin_some(node->get_node_base_interface());
      result["unsmoothed_available"] = static_cast<bool>(unsmoothed);
      result["unsmoothed_message_count"] = raw_messages;
      result["unsmoothed_source"] = "Installed plugin unsmoothed_plan lifecycle publisher via intra-process subscription";
      if (unsmoothed) {
        writePath(prefix+".unsmoothed.path", *unsmoothed);
        result["unsmoothed_pose_count"] = unsmoothed->poses.size();
        result["smoothing_changed_pose_geometry"] = unsmoothed->poses.size() != path.poses.size();
        if (unsmoothed->poses.size() == path.poses.size()) {
          for (size_t i = 0; i < path.poses.size(); ++i) {
            if (unsmoothed->poses[i].pose.position != path.poses[i].pose.position) {
              result["smoothing_changed_pose_geometry"] = true;
            }
          }
        }
      } else {
        result["unsmoothed_unavailable_reason"] = "No debug message received; no planner-source or smoother changes made";
      }
      // Release observed messages before plugin cleanup / class-loader DSO teardown.
      unsmoothed.reset(); raw_sub.reset();
      snapshot(prefix+".costmap-after-plan.u8", master);
      plugin->deactivate(); plugin->cleanup(); plugin.reset(); cm.reset(); node.reset();
      result["lifecycle_cleanup_success"] = true;
    }
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    result["runtime_exception"] = e.what();
    std::cerr << "REPLAY_RUNTIME_EXCEPTION: " << e.what() << std::endl;
    if (rclcpp::ok()) {rclcpp::shutdown();}
    rc = 1;
  }
  std::ofstream(prefix+".result.json") << result.dump(2) << '\n';
  return rc;
}
