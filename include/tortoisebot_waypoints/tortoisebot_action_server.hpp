#ifndef TORTOISEBOT_ACTION_SERVER_H
#define TORTOISEBOT_ACTION_SERVER_H

#include "cmath"
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl/publisher.h"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tortoisebot_waypoints_interface/action/detail/waypoint_action__struct.hpp"
#include "tortoisebot_waypoints_interface/action/waypoint_action.hpp"
#include <cmath>
#include <memory>
#include <string>

class WaypointActionClass : public rclcpp::Node {

public:
  using WaypointAction =
      tortoisebot_waypoints_interface::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ServerGoalHandle<WaypointAction>;
  using Odometry = nav_msgs::msg::Odometry;
  using Vel = geometry_msgs::msg::Twist;
  using Point = geometry_msgs::msg::Point;

  explicit WaypointActionClass(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~WaypointActionClass();

private:
  rclcpp_action::Server<WaypointAction>::SharedPtr action_srv_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<Vel>::SharedPtr vel_pub_;

  void odom_callback(const Odometry::SharedPtr msg);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  // robot state var
  Point _position;
  double _yaw;
  // machine state
  std::string _state;

  // goal
  Point _des_pos;

  // parameters
  const double _yaw_precision = 2*M_PI / 90;
  const double _dist_precision = 0.05;
};

#endif