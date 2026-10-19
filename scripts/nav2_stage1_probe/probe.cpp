// Offline installed-library probe. Deliberately no rclcpp::init, nodes or DDS.
#include <iostream>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/controller.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <dwb_core/illegal_trajectory_tracker.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/parameter_map.hpp>

using Json = nlohmann::json;

Json value(const rclcpp::Parameter & p)
{
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER: return p.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return p.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING: return p.as_string();
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: return p.as_byte_array();
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: return p.as_bool_array();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: return p.as_integer_array();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: return p.as_double_array();
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: return p.as_string_array();
    default: throw std::runtime_error("unsupported parameter type");
  }
}

int main(int argc, char ** argv)
{
  try {
    Json result = Json::object();
    if (argc >= 4 && std::string(argv[1]) == "--parameters") {
      const std::string target = "/" + std::string(argv[2]);
      for (int i = 3; i < argc; ++i) {
        // These launch files use exact node names and /** generated dictionaries.
        // Installed rclcpp/rcl_yaml_param_parser flatten nested ROS parameters.
        for (const auto & entry : rclcpp::parameter_map_from_yaml_file(argv[i])) {
          if (entry.first != target && entry.first != "/**") {continue;}
          for (const auto & p : entry.second) {result[p.get_name()] = value(p);}
        }
      }
    } else if (argc == 2 && std::string(argv[1]) == "--plugins") {
      pluginlib::ClassLoader<nav2_core::GlobalPlanner> planners("nav2_core", "nav2_core::GlobalPlanner");
      for (const std::string name : {"nav2_navfn_planner/NavfnPlanner", "nav2_smac_planner/SmacPlanner2D"}) {
        auto instance = planners.createUniqueInstance(name);
        result["planners"].push_back(name);
        result["libraries"][name] = planners.getClassLibraryPath(name);
      }
      pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core", "nav2_core::Controller");
      const std::string dwb = "dwb_core::DWBLocalPlanner";
      {auto instance = controllers.createUniqueInstance(dwb);}
      result["controller"] = dwb;
      result["libraries"][dwb] = controllers.getClassLibraryPath(dwb);
      pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critics("dwb_core", "dwb_core::TrajectoryCritic");
      const std::string gate = "dwb_critics::ObstacleFootprintCritic";
      {auto instance = critics.createUniqueInstance(gate);}
      result["footprint_critic"] = gate;
      result["libraries"][gate] = critics.getClassLibraryPath(gate);
      dwb_core::IllegalTrajectoryTracker tracker;
      tracker.addIllegalTrajectory(dwb_core::IllegalTrajectoryException("BaseObstacle", "Trajectory Hits Obstacle."));
      const auto reasons = tracker.getPercentages();
      result["rejection_reason"] = reasons.begin()->first.second;
      result["reported_fraction"] = reasons.begin()->second;
    } else {
      throw std::runtime_error("usage: --plugins | --parameters NODE YAML [YAML ...]");
    }
    // All plugin instances are destroyed before this final machine-readable line.
    std::cout << result.dump() << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
