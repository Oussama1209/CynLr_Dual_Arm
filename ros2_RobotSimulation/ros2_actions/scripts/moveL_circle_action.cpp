/*

  MoveL_Circle ACTION SERVER (direct MoveIt, smooth Cartesian circle)
  MoveL_Lissajous ACTION SERVER (direct MoveIt)

  Based on the IFRA-Cranfield "MoveL" action server structure

*/

#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros2_data/action/move_l.hpp"

// Simple constants
const double pi = 3.14159265358979323846;

// GLOBAL: robot / planning group parameter (same pattern as MoveL)
std::string my_param = "none";

// Small helper node to read ROB_PARAM (same as in MoveL)
class ros2_RobotTrigger : public rclcpp::Node
{
public:
  ros2_RobotTrigger()
  : Node("ros2_RobotTrigger_PARAM")
  {
    // Declare + get ROB_PARAM
    this->declare_parameter("ROB_PARAM");
    my_param = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", my_param.c_str());
  }
};

// GLOBAL: MoveIt2 MoveGroupInterface (same model as MoveL)
moveit::planning_interface::MoveGroupInterface move_group_interface;

class MoveLCircleActionServer : public rclcpp::Node
{
public:
  using MoveL      = ros2_data::action::MoveL;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveL>;

  explicit MoveLCircleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("MoveL_Circle_ActionServer", options),
    DEFAULT_NUM_POINTS_(60),
    DEFAULT_EEF_STEP_(0.005)
  {

    // ROB_PARAM – planning group name (e.g. "arm" or "panda_arm")
    if (!this->has_parameter("ROB_PARAM")) {
      this->declare_parameter<std::string>("ROB_PARAM", "none");
    }
    this->get_parameter("ROB_PARAM", group_name_);
    RCLCPP_INFO(this->get_logger(), "MoveL_Circle: ROB_PARAM = '%s'", group_name_.c_str());

    // num_points – number of *circle waypoints* (before time-discretization)
    if (!this->has_parameter("num_points")) {
      this->declare_parameter<int>("num_points", DEFAULT_NUM_POINTS_);
    }
    this->get_parameter("num_points", num_points_);
    if (num_points_ < 4) {
      RCLCPP_WARN(this->get_logger(),
                  "num_points (%d) < 4, clamping to 4.", num_points_);
      num_points_ = 4;
    }

    // eef_step – step size for computeCartesianPath (meters)
    if (!this->has_parameter("eef_step")) {
      this->declare_parameter<double>("eef_step", DEFAULT_EEF_STEP_);
    }
    this->get_parameter("eef_step", eef_step_);
    if (eef_step_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "eef_step (%.6f) <= 0, clamping to 0.001.", eef_step_);
      eef_step_ = 0.001;
    }

    // circle_time – nominal duration (seconds) for one revolution at speed=1.0
    if (!this->has_parameter("circle_time")) {
      this->declare_parameter<double>("circle_time", 6.0);
    }
    this->get_parameter("circle_time", circle_time_);
    if (circle_time_ <= 0.1) {
      RCLCPP_WARN(this->get_logger(),
                  "circle_time (%.3f) too small, clamping to 1.0.", circle_time_);
      circle_time_ = 1.0;
    }

    action_server_ = rclcpp_action::create_server<MoveL>(
      this,
      "MoveL_Circle",
      std::bind(&MoveLCircleActionServer::handle_goal,     this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveLCircleActionServer::handle_cancel,   this,
                std::placeholders::_1),
      std::bind(&MoveLCircleActionServer::handle_accepted, this,
                std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "MoveL_Circle_ActionServer for group '%s' is up "
                "(num_points=%d, eef_step=%.4f, circle_time=%.2f).",
                group_name_.c_str(), num_points_, eef_step_, circle_time_);
  }

private:
  // Constants
  const int    DEFAULT_NUM_POINTS_;
  const double DEFAULT_EEF_STEP_;

  // Params
  int         num_points_;
  double      eef_step_;
  double      circle_time_;
  std::string group_name_;

  // Action server
  rclcpp_action::Server<MoveL>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const MoveL::Goal> goal)
  {
    double moveX = goal->movex;
    double moveY = goal->movey;
    double moveZ = goal->movez;

    RCLCPP_INFO(
      this->get_logger(),
      "MoveL_Circle: received goal (movex=%.3f -> radius, movey=%.3f, movez=%.3f, speed=%.3f)",
      moveX, moveY, moveZ, goal->speed);

    double radius = std::fabs(moveX);
    if (radius < 1e-3) {
      RCLCPP_WARN(this->get_logger(),
                  "MoveL_Circle: |movex| too small (%.6f). Radius must be > 0.", radius);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "MoveL_Circle: cancel requested.");
    // Stop any active trajectory on this MoveGroup
    move_group_interface.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Run execute() in a separate thread as recommended in rclcpp_action
    std::thread(
      [this, goal_handle]()
      {
        execute(goal_handle);
      }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(),
                "MoveL_Circle: starting execution (direct MoveIt).");

    auto result = std::make_shared<MoveL::Result>();

    try {
      const auto goal = goal_handle->get_goal();

      double radius = std::fabs(goal->movex);
      double speed  = goal->speed;

      // Direction: sign(movex) -> + = CCW, - = CW
      double direction = (goal->movex >= 0.0) ? 1.0 : -1.0;

      // Validate radius
      if (radius < 1e-3) {
        RCLCPP_WARN(this->get_logger(),
                    "MoveL_Circle: |movex| too small (%.6f). Radius must be > 0.", radius);
        result->result = "MoveL_Circle:INVALID-RADIUS";
        goal_handle->succeed(result);
        return;
      }

      // Clamp speed
      if (speed <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "MoveL_Circle: speed <= 0.0 (%.3f). Clamping to 0.1.", speed);
        speed = 0.1;
      } else if (speed > 5.0) {
        RCLCPP_WARN(this->get_logger(),
                    "MoveL_Circle: speed > 5.0 (%.3f). Clamping to 1.0.", speed);
        speed = 5.0;
      }

      // Ensure group exists
      const moveit::core::JointModelGroup * joint_model_group =
          move_group_interface.getCurrentState()->getJointModelGroup(my_param);
      if (!joint_model_group) {
        RCLCPP_ERROR(this->get_logger(),
                     "MoveL_Circle: JointModelGroup '%s' not found.", my_param.c_str());
        result->result = "MoveL_Circle:INVALID-GROUP";
        goal_handle->succeed(result);
        return;
      }

      // Get current TCP pose (uses group's end-effector)
      auto current_pose = move_group_interface.getCurrentPose();
      geometry_msgs::msg::Pose start_pose = current_pose.pose;

      RCLCPP_INFO(this->get_logger(),
                  "MoveL_Circle: start pose = (x=%.3f, y=%.3f, z=%.3f)",
                  start_pose.position.x,
                  start_pose.position.y,
                  start_pose.position.z);

      // Build circle waypoints in YZ-plane around current TCP (x fixed)
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.reserve(static_cast<std::size_t>(num_points_) + 1);

      const double two_pi = 2.0 * pi;
      const int N = num_points_;

      // Center such that start pose is on the circle (yz-plane)
      const double center_y = start_pose.position.y;
      const double center_z = start_pose.position.z - radius;

      for (int i = 0; i <= N; ++i) {
        double theta = direction * two_pi * static_cast<double>(i) / static_cast<double>(N);

        geometry_msgs::msg::Pose p = start_pose;
        p.position.y = center_y + radius * std::sin(theta);
        p.position.z = center_z + radius * std::cos(theta);
        waypoints.push_back(p);
      }

      RCLCPP_INFO(this->get_logger(),
                  "MoveL_Circle: planning circular path with N=%d waypoints, "
                  "radius=%.3f, eef_step=%.4f, dir=%s",
                  N, radius, eef_step_,
                  (direction > 0.0 ? "CCW" : "CW"));

      // Compute Cartesian path (MoveIt will discretize further internally)
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;

      double fraction = move_group_interface.computeCartesianPath(
          waypoints, eef_step_, jump_threshold, trajectory);

      if (fraction < 0.99) {
        RCLCPP_WARN(this->get_logger(),
                    "MoveL_Circle: computeCartesianPath followed only %.1f %% of the path.",
                    fraction * 100.0);
        result->result = "MoveL_Circle:PLANNING-FRACTION-LOW";
        goal_handle->succeed(result);  // same convention as MoveL
        return;
      }

      {
        auto & points = trajectory.joint_trajectory.points;
        const std::size_t num_pts = points.size();

        if (num_pts < 2) {
          RCLCPP_ERROR(this->get_logger(),
                       "MoveL_Circle: trajectory has < 2 points, aborting.");
          result->result = "MoveL_Circle:TOO-FEW-POINTS";
          goal_handle->succeed(result);
          return;
        }

        // One revolution duration: circle_time_ scaled by 1/speed
        double total_time = circle_time_ / speed;

        for (std::size_t i = 0; i < num_pts; ++i) {
          double alpha = static_cast<double>(i) /
                         static_cast<double>(num_pts - 1);  // in [0,1]
          double t = total_time * alpha;

          int32_t sec = static_cast<int32_t>(std::floor(t));
          uint32_t nsec = static_cast<uint32_t>((t - static_cast<double>(sec)) * 1e9);

          points[i].time_from_start.sec = sec;
          points[i].time_from_start.nanosec = nsec;
        }

        RCLCPP_INFO(this->get_logger(),
                    "MoveL_Circle: time-parameterized %zu points over %.2f s.",
                    num_pts, total_time);
      }

      // If goal got cancelled before execution – stop here
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(),
                    "MoveL_Circle: goal canceled before execution.");
        move_group_interface.stop();
        result->result = "MoveL_Circle:CANCELED";
        goal_handle->canceled(result);
        return;
      }

      // Execute the single smooth trajectory
      auto err = move_group_interface.execute(trajectory);

      if (err == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(),
                    "%s - MoveL_Circle: Execution successful!",
                    my_param.c_str());
        result->result = "MoveL_Circle:SUCCESS";
        goal_handle->succeed(result);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "%s - MoveL_Circle: Execution failed (error code %d).",
                    my_param.c_str(), err.val);
        result->result = "MoveL_Circle:EXECUTION-FAILED";
        goal_handle->succeed(result);
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "MoveL_Circle: unhandled std::exception: %s", e.what());
      result->result = "MoveL_Circle:EXCEPTION";
      goal_handle->succeed(result);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(),
                   "MoveL_Circle: unknown non-std exception.");
      result->result = "MoveL_Circle:EXCEPTION";
      goal_handle->succeed(result);
    }
  }
};

class MoveLLissajousActionServer : public rclcpp::Node
{
public:
  using MoveL      = ros2_data::action::MoveL;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveL>;

  explicit MoveLLissajousActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("MoveL_Lissajous_ActionServer", options),
    DEFAULT_NUM_POINTS_(120),
    DEFAULT_EEF_STEP_(0.005)
  {
    // ROB_PARAM – planning group name
    if (!this->has_parameter("ROB_PARAM")) {
      this->declare_parameter<std::string>("ROB_PARAM", "none");
    }
    this->get_parameter("ROB_PARAM", group_name_);
    RCLCPP_INFO(this->get_logger(), "MoveL_Lissajous: ROB_PARAM = '%s'", group_name_.c_str());

    // num_points – number of curve waypoints
    if (!this->has_parameter("num_points")) {
      this->declare_parameter<int>("num_points", DEFAULT_NUM_POINTS_);
    }
    this->get_parameter("num_points", num_points_);
    if (num_points_ < 10) {
      RCLCPP_WARN(this->get_logger(), "num_points (%d) < 10, clamping to 10.", num_points_);
      num_points_ = 10;
    }

    // eef_step – step size for computeCartesianPath (meters)
    if (!this->has_parameter("eef_step")) {
      this->declare_parameter<double>("eef_step", DEFAULT_EEF_STEP_);
    }
    this->get_parameter("eef_step", eef_step_);
    if (eef_step_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "eef_step (%.6f) <= 0, clamping to 0.001.", eef_step_);
      eef_step_ = 0.001;
    }

    // curve_time – nominal duration (seconds) for one complete Lissajous trace at speed=1.0
    if (!this->has_parameter("curve_time")) {
      this->declare_parameter<double>("curve_time", 6.0);
    }
    this->get_parameter("curve_time", curve_time_);
    if (curve_time_ <= 0.1) {
      RCLCPP_WARN(this->get_logger(), "curve_time (%.3f) too small, clamping to 1.0.", curve_time_);
      curve_time_ = 1.0;
    }

    // Lissajous default frequencies (a,b) -> a=1, b=2
    if (!this->has_parameter("liss_a")) {
      this->declare_parameter<int>("liss_a", 1);
    }
    if (!this->has_parameter("liss_b")) {
      this->declare_parameter<int>("liss_b", 2);
    }
    this->get_parameter("liss_a", liss_a_);
    this->get_parameter("liss_b", liss_b_);
    if (liss_a_ < 1) liss_a_ = 1;
    if (liss_b_ < 1) liss_b_ = 1;

    // Phase offsets (to avoid starting exactly at the crossing if you want)
    if (!this->has_parameter("delta_y")) {
      this->declare_parameter<double>("delta_y", pi/2.0);   // shifts start point
    }
    if (!this->has_parameter("delta_z")) {
      this->declare_parameter<double>("delta_z", 0.0);
    }
    this->get_parameter("delta_y", delta_y_);
    this->get_parameter("delta_z", delta_z_);

    // num_cycles: repeat the full closed curve this many times (default 1)
    if (!this->has_parameter("num_cycles")) {
      this->declare_parameter<int>("num_cycles", 1);
    }
    this->get_parameter("num_cycles", num_cycles_);
    if (num_cycles_ < 1) num_cycles_ = 1;

    // Action server
    action_server_ = rclcpp_action::create_server<MoveL>(
      this,
      "MoveL_Lissajous",
      std::bind(&MoveLLissajousActionServer::handle_goal,     this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveLLissajousActionServer::handle_cancel,   this,
                std::placeholders::_1),
      std::bind(&MoveLLissajousActionServer::handle_accepted, this,
                std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "MoveL_Lissajous_ActionServer is up (num_points=%d, eef_step=%.4f, curve_time=%.2f, a=%d, b=%d, cycles=%d).",
                num_points_, eef_step_, curve_time_, liss_a_, liss_b_, num_cycles_);
  }

private:
  // Constants
  const int    DEFAULT_NUM_POINTS_;
  const double DEFAULT_EEF_STEP_;

  // Params
  int         num_points_;
  double      eef_step_;
  double      curve_time_;
  int         liss_a_;
  int         liss_b_;
  double      delta_y_;
  double      delta_z_;
  int         num_cycles_;
  std::string group_name_;

  // Action server
  rclcpp_action::Server<MoveL>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const MoveL::Goal> goal)
  {
    const double radius = std::fabs(goal->movex);
    RCLCPP_INFO(this->get_logger(),
                "MoveL_Lissajous: received goal (movex=%.3f -> amplitude, movey=%.3f, movez=%.3f, speed=%.3f)",
                goal->movex, goal->movey, goal->movez, goal->speed);

    if (radius < 1e-3) {
      RCLCPP_WARN(this->get_logger(),
                  "MoveL_Lissajous: |movex| too small (%.6f). Amplitude must be > 0.", radius);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "MoveL_Lissajous: cancel requested.");
    move_group_interface.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "MoveL_Lissajous: starting execution (direct MoveIt).");
    auto result = std::make_shared<MoveL::Result>();

    try {
      const auto goal = goal_handle->get_goal();

      double A      = std::fabs(goal->movex);   // amplitude
      double speed  = goal->speed;

      // Optional: use sign(movex) to reverse traversal direction
      double direction = (goal->movex >= 0.0) ? 1.0 : -1.0;

      // Clamp speed (same idea as your circle)
      if (speed <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "MoveL_Lissajous: speed <= 0.0 (%.3f). Clamping to 0.1.", speed);
        speed = 0.1;
      } else if (speed > 5.0) {
        RCLCPP_WARN(this->get_logger(), "MoveL_Lissajous: speed > 5.0 (%.3f). Clamping to 5.0.", speed);
        speed = 5.0;
      }

      // Frequencies: by default from params (a=1,b=2 gives a 2-lobe figure-eight).
      // If user provides movey/movez as >=1, override with rounded ints.
      int a = liss_a_;
      int b = liss_b_;
      if (std::fabs(goal->movey) >= 1.0) a = std::max(1, static_cast<int>(std::lround(std::fabs(goal->movey))));
      if (std::fabs(goal->movez) >= 1.0) b = std::max(1, static_cast<int>(std::lround(std::fabs(goal->movez))));

      // Ensure group exists (kept consistent with your circle code using my_param)
      const moveit::core::JointModelGroup * joint_model_group =
          move_group_interface.getCurrentState()->getJointModelGroup(my_param);
      if (!joint_model_group) {
        RCLCPP_ERROR(this->get_logger(),
                     "MoveL_Lissajous: JointModelGroup '%s' not found.", my_param.c_str());
        result->result = "MoveL_Lissajous:INVALID-GROUP";
        goal_handle->succeed(result);
        return;
      }

      // Start pose
      auto current_pose = move_group_interface.getCurrentPose();
      geometry_msgs::msg::Pose start_pose = current_pose.pose;

      const double cy = start_pose.position.y - A * std::sin(delta_y_);
      const double cz = start_pose.position.z - A * std::sin(delta_z_);

      // Waypoints for a closed Lissajous
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.reserve(static_cast<std::size_t>(num_points_) + 1);

      const double two_pi = 2.0 * pi;
      const int N = num_points_;
      const double T = two_pi * static_cast<double>(num_cycles_);

      for (int i = 0; i <= N; ++i) {
        double u = static_cast<double>(i) / static_cast<double>(N);   // 0..1
        double t = direction * (T * u);

        geometry_msgs::msg::Pose p = start_pose;
        p.position.y = cy + A * std::sin(static_cast<double>(a) * t + delta_y_);
        p.position.z = cz + A * std::sin(static_cast<double>(b) * t + delta_z_);
        waypoints.push_back(p);
      }

      RCLCPP_INFO(this->get_logger(),
                  "MoveL_Lissajous: planning with N=%d waypoints, A=%.3f, eef_step=%.4f, a=%d, b=%d, cycles=%d",
                  N, A, eef_step_, a, b, num_cycles_);

      // Compute Cartesian path
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;

      double fraction = move_group_interface.computeCartesianPath(
          waypoints, eef_step_, jump_threshold, trajectory);

      if (fraction < 0.99) {
        RCLCPP_WARN(this->get_logger(),
                    "MoveL_Lissajous: computeCartesianPath followed only %.1f %% of the path.",
                    fraction * 100.0);
        result->result = "MoveL_Lissajous:PLANNING-FRACTION-LOW";
        goal_handle->succeed(result);
        return;
      }

      // Time parameterization
      {
        auto & points = trajectory.joint_trajectory.points;
        const std::size_t num_pts = points.size();

        if (num_pts < 2) {
          RCLCPP_ERROR(this->get_logger(),
                       "MoveL_Lissajous: trajectory has < 2 points, aborting.");
          result->result = "MoveL_Lissajous:TOO-FEW-POINTS";
          goal_handle->succeed(result);
          return;
        }

        double total_time = curve_time_ / speed;

        for (std::size_t i = 0; i < num_pts; ++i) {
          double alpha = static_cast<double>(i) / static_cast<double>(num_pts - 1);
          double tt = total_time * alpha;

          int32_t sec = static_cast<int32_t>(std::floor(tt));
          uint32_t nsec = static_cast<uint32_t>((tt - static_cast<double>(sec)) * 1e9);

          points[i].time_from_start.sec = sec;
          points[i].time_from_start.nanosec = nsec;
        }

        RCLCPP_INFO(this->get_logger(),
                    "MoveL_Lissajous: time-parameterized %zu points over %.2f s.",
                    num_pts, total_time);
      }

      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "MoveL_Lissajous: goal canceled before execution.");
        move_group_interface.stop();
        result->result = "MoveL_Lissajous:CANCELED";
        goal_handle->canceled(result);
        return;
      }

      // Execute
      auto err = move_group_interface.execute(trajectory);

      if (err == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s - MoveL_Lissajous: Execution successful!", my_param.c_str());
        result->result = "MoveL_Lissajous:SUCCESS";
        goal_handle->succeed(result);
      } else {
        RCLCPP_WARN(this->get_logger(), "%s - MoveL_Lissajous: Execution failed (error code %d).",
                    my_param.c_str(), err.val);
        result->result = "MoveL_Lissajous:EXECUTION-FAILED";
        goal_handle->succeed(result);
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "MoveL_Lissajous: std::exception: %s", e.what());
      result->result = "MoveL_Lissajous:EXCEPTION";
      goal_handle->succeed(result);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "MoveL_Lissajous: unknown exception.");
      result->result = "MoveL_Lissajous:EXCEPTION";
      goal_handle->succeed(result);
    }
  }
};


int main(int argc, char ** argv)
{
  // Initialise ROS2
  rclcpp::init(argc, argv);

  // Read ROB_PARAM
  auto node_PARAM = std::make_shared<ros2_RobotTrigger>();
  rclcpp::spin_some(node_PARAM);

  // Create dedicated node for MoveGroupInterface
  auto name      = std::string("_MoveL_interface");
  auto node2name = my_param + name;

  auto const node2 = std::make_shared<rclcpp::Node>(
      node2name,
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node2);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create MoveGroupInterface + PlanningSceneInterface
  using moveit::planning_interface::MoveGroupInterface;
  move_group_interface = MoveGroupInterface(node2, my_param);

  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();
  (void)planning_scene_interface;  // not used but kept for consistency

  // Start the MoveL_Circle action server node
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto circle_server    = std::make_shared<MoveLCircleActionServer>(options);
  auto lissajous_server = std::make_shared<MoveLLissajousActionServer>(options);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(circle_server);
  exec.add_node(lissajous_server);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
