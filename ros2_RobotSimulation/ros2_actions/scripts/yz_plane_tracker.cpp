#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


const double PI = 3.14159265358979323846;

std::string g_robot_group = "none";

class RobotParamNode : public rclcpp::Node
{
public:
  RobotParamNode() : Node("yz_plane_tracker_PARAM")
  {
    this->declare_parameter("ROB_PARAM");
    g_robot_group = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "ROB_PARAM received in yz_plane_tracker -> %s", g_robot_group.c_str());
  }
};

// Global MoveIt interface
moveit::planning_interface::MoveGroupInterface g_move_group_interface;

class YZPlaneTracker : public rclcpp::Node
{
public:
  explicit YZPlaneTracker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("yz_plane_tracker", options)
  {
    //Parameters (tune from launch if you want)
    this->declare_parameter<double>("gain_y",    0.05);
    this->declare_parameter<double>("gain_z",    0.05);
    this->declare_parameter<double>("max_step",  0.02);
    this->declare_parameter<double>("deadband",  0.02);
    this->declare_parameter<double>("eef_step",  0.005);
    this->declare_parameter<double>("control_rate", 10.0);
    this->declare_parameter<double>("vel_scale", 0.5);
    this->declare_parameter<double>("acc_scale", 0.5);
    this->declare_parameter<double>("segment_time", 0.5);

    this->get_parameter("gain_y", gain_y_);
    this->get_parameter("gain_z", gain_z_);
    this->get_parameter("max_step", max_step_);
    this->get_parameter("deadband", deadband_);
    this->get_parameter("eef_step", eef_step_);
    this->get_parameter("control_rate", control_rate_);
    this->get_parameter("vel_scale", vel_scale_);
    this->get_parameter("acc_scale", acc_scale_);
    this->get_parameter("segment_time", segment_time_);

    RCLCPP_INFO(this->get_logger(),
                "YZPlaneTracker for group '%s' started (waiting for service call).",
                g_robot_group.c_str());

    //Subscriber to detected ball
    ball_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/detected_ball",
      1,
      std::bind(&YZPlaneTracker::ballCallback, this, std::placeholders::_1)
    );

    //Service to enable/disable tracking
    enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "enable_tracking",
      std::bind(&YZPlaneTracker::handleEnableTracking,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    //Timer control loop
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&YZPlaneTracker::controlLoop, this)
    );
  }

private:
  // Parameters
  double gain_y_{0.05};
  double gain_z_{0.05};
  double max_step_{0.02};
  double deadband_{0.02};
  double eef_step_{0.005};
  double control_rate_{10.0};
  double vel_scale_{0.5};
  double acc_scale_{0.5};
  double segment_time_{0.5};

  // State from vision
  geometry_msgs::msg::Point last_target_;
  rclcpp::Time last_target_time_;
  bool have_target_{false};

  // Tracking gate (service-controlled)
  bool tracking_enabled_{false};

  // ROS objects
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Service callback: enable/disable tracking
  void handleEnableTracking(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    tracking_enabled_ = req->data;

    have_target_ = false;

    res->success = true;
    res->message = tracking_enabled_ ? "Tracking ENABLED" : "Tracking DISABLED";

    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }

  // Callback: receive normalized target (x,y in [-1,1])
  void ballCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    last_target_ = *msg;
    last_target_time_ = this->now();
    have_target_ = true;
  }

  // Simple time-stamp assignment so controller accepts trajectory  
  void assignMonotonicTimestamps(moveit_msgs::msg::RobotTrajectory & traj,
                                 double total_time_sec)
  {
    auto & points = traj.joint_trajectory.points;
    if (points.size() < 2) {
      return;
    }

    if (total_time_sec <= 0.0) {
      total_time_sec = 0.5;
    }

    const size_t N = points.size();
    const double dt = total_time_sec / static_cast<double>(N - 1);
    double t = 0.0;

    for (size_t i = 0; i < N; ++i) {
      int32_t sec = static_cast<int32_t>(std::floor(t));
      uint32_t nsec = static_cast<uint32_t>((t - static_cast<double>(sec)) * 1e9);
      points[i].time_from_start.sec = sec;
      points[i].time_from_start.nanosec = nsec;
      t += dt;
    }
  }

  // Control loop
  void controlLoop()
  {
    if (!tracking_enabled_) {
      return;
    }

    if (!have_target_) {
      return;
    }

    const double max_age = 0.5;
    if ((this->now() - last_target_time_).seconds() > max_age) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Target is stale, not moving.");
      have_target_ = false;
      return;
    }

    double ex = last_target_.x;
    double ey = last_target_.y;

    if (std::fabs(ex) < deadband_ && std::fabs(ey) < deadband_) {
      return;
    }

    geometry_msgs::msg::Pose current_pose =
      g_move_group_interface.getCurrentPose().pose;

    double dY = -gain_y_ * ex;
    double dZ = -gain_z_ * ey;

    double norm = std::sqrt(dY * dY + dZ * dZ);
    if (norm > max_step_) {
      double scale = max_step_ / norm;
      dY *= scale;
      dZ *= scale;
    }

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.y += dY;
    target_pose.position.z += dZ;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory traj;
    const double jump_threshold = 0.0;

    double fraction = g_move_group_interface.computeCartesianPath(
      waypoints, eef_step_, jump_threshold, traj);

    if (fraction < 0.9) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "computeCartesianPath fraction too low: %.2f, skipping segment", fraction);
      return;
    }

    assignMonotonicTimestamps(traj, segment_time_);

    g_move_group_interface.setMaxVelocityScalingFactor(vel_scale_);
    g_move_group_interface.setMaxAccelerationScalingFactor(acc_scale_);

    auto ec = g_move_group_interface.execute(traj);
    if (!ec) {
      RCLCPP_WARN(this->get_logger(),
                  "%s - YZPlaneTracker: Trajectory execution failed with code %d",
                  g_robot_group.c_str(), ec.val);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_param = std::make_shared<RobotParamNode>();
  rclcpp::spin_some(node_param);

  auto node2 = std::make_shared<rclcpp::Node>(
    g_robot_group + std::string("_yz_plane_interface"),
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node2);
  std::thread([&executor]() { executor.spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  g_move_group_interface = MoveGroupInterface(node2, g_robot_group);

  auto tracker = std::make_shared<YZPlaneTracker>();
  rclcpp::spin(tracker);

  rclcpp::shutdown();
  return 0;
}
