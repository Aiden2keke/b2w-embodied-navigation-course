// Boundary harness: robot pose, costmap getters and zero odometry are supplied
// without lifecycle nodes. Transform and goal-checker logic are real ROS libraries.
// Removing the production guard must fail the stale-transform and latch assertions.
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "tf2_ros/buffer.h"

class Checker : public nav2_controller::SimpleGoalChecker {
public:
  int calls = 0;
  Checker() {yaw_goal_tolerance_ = .30;}
  bool xyPending() const {return check_xy_;}
  bool isGoalReached(const geometry_msgs::msg::Pose & q,
    const geometry_msgs::msg::Pose & g, const geometry_msgs::msg::Twist & v) override
  {
    ++calls;
    return SimpleGoalChecker::isGoalReached(q, g, v);
  }
};
struct CostmapBoundary {
  std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  double getTransformTolerance() const {return .5;}
  std::string getGlobalFrameID() const {return "odom";}
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() {return buffer;}
};
struct OdomBoundary {
  nav_2d_msgs::msg::Twist2D getTwist() {return nav_2d_msgs::msg::Twist2D();}
};
class ControllerServer {
public:
  bool isGoalReached();
  geometry_msgs::msg::PoseStamped query, end_pose_;
  bool pose_ok = true;
  std::shared_ptr<CostmapBoundary> costmap_ros_ = std::make_shared<CostmapBoundary>();
  std::unique_ptr<OdomBoundary> odom_sub_ = std::make_unique<OdomBoundary>();
  std::shared_ptr<Checker> checker = std::make_shared<Checker>();
  std::map<std::string, std::shared_ptr<Checker>> goal_checkers_{{"goal", checker}};
  std::string current_goal_checker_ = "goal";
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) {pose = query; return pose_ok;}
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(nav_2d_msgs::msg::Twist2D v) {return v;}
};
#include "goal_method.inc"

int failures = 0;
void check(bool condition, const char * name) {
  std::cout << (condition ? "PASS: " : "FAIL: ") << name << '\n';
  if (!condition) {++failures;}
}
int main() {
  ControllerServer server;
  auto tf = geometry_msgs::msg::TransformStamped();
  tf.header.frame_id = "odom";
  tf.child_frame_id = "map";
  tf.header.stamp = rclcpp::Time(57100000000LL);
  tf.transform.translation.x = 2.;
  tf.transform.rotation.w = 1.;
  check(server.costmap_ros_->buffer->setTransform(tf, "offline", false), "synthetic TF installed");
  // Buffer is immutable and prepopulated: no ROS init, listener, DDS or executor.
  server.costmap_ros_->buffer->setUsingDedicatedThread(true);
  server.end_pose_.header.frame_id = "map";
  server.end_pose_.header.stamp = rclcpp::Time(57707500000LL);
  server.end_pose_.pose.position.y = 7.19;
  server.query.pose.position.x = .1;
  check(!server.isGoalReached(), "stale endpoint transform rejects near odom origin");
  check(server.checker->calls == 0, "failed transform never invokes GoalChecker");
  check(server.checker->xyPending(), "failed transform cannot latch XY success");

  server.checker->reset(); server.checker->calls = 0;
  server.end_pose_.header.stamp = tf.header.stamp;
  check(!server.isGoalReached(), "successful nonidentity transform of distant goal rejects");
  check(server.checker->calls == 1, "successful transform reaches GoalChecker");

  server.checker->reset(); server.checker->calls = 0;
  server.query.pose.position.x = 2.1; server.query.pose.position.y = 7.19;
  check(server.isGoalReached(), "legitimate arrival at transformed endpoint succeeds");
  check(server.checker->calls == 1, "arrival checked exactly once");

  server.checker->reset(); server.query.pose.orientation.z = std::sin(.5);
  server.query.pose.orientation.w = std::cos(.5);
  check(!server.isGoalReached(), "arrival with wrong heading rejects");

  server.checker->reset(); server.query.pose.orientation.z = 0.; server.query.pose.orientation.w = 1.;
  server.end_pose_.header.frame_id = "odom"; server.end_pose_.pose = server.query.pose;
  check(server.isGoalReached(), "same-frame legitimate arrival succeeds");
  server.checker->reset(); server.checker->calls = 0; server.pose_ok = false;
  check(!server.isGoalReached() && server.checker->calls == 0, "missing robot pose rejects before checker");
  std::cout << "Failures: " << failures << '\n';
  return failures ? 1 : 0;
}
