#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tortoisebot_waypoints_interface/action/waypoint_action.hpp"
#include <cmath>
#include <functional>
#include <math.h>
#include <memory>
#include <tortoisebot_waypoints/tortoisebot_action_server.hpp>

WaypointActionClass::WaypointActionClass(const rclcpp::NodeOptions &options)
    : Node("tortoisebot_action_server", options) {

  //////////////////////////////////
  using namespace std::placeholders;
  this->action_srv_ = rclcpp_action::create_server<WaypointAction>(
      this, "tortoisebot_as",
      std::bind(&WaypointActionClass::handle_goal, this, _1, _2),
      std::bind(&WaypointActionClass::handle_cancel, this, _1),
      std::bind(&WaypointActionClass::handle_accepted, this, _1));

  this->odom_sub_ = this->create_subscription<Odometry>(
      "odom", 10, std::bind(&WaypointActionClass::odom_callback, this, _1));
  this->vel_pub_ = this->create_publisher<Vel>("cmd_vel", 10);

  // initialize variables
  _yaw = 0;
  _state = "idle";

  RCLCPP_INFO(this->get_logger(), "Waypoint Action Server Started..");
}

WaypointActionClass::~WaypointActionClass() {}

void WaypointActionClass::odom_callback(const Odometry::SharedPtr msg) {
  // position
  this->_position = msg->pose.pose.position;

  // yaw
  auto quaternion = tf2::Quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // convert to RPY
  tf2::Matrix3x3 m(quaternion);
  double r, p;
  m.getRPY(r, p, this->_yaw);
}

rclcpp_action::GoalResponse WaypointActionClass::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const WaypointAction::Goal> goal) {

  ///////////////////////////////
  RCLCPP_INFO(this->get_logger(), "Received goal request: [x: %f, y: %f]", goal->position.x, goal->position.y);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointActionClass::handle_cancel(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

  /////////////////////////////////
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointActionClass::handle_accepted(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

  ///////////////////////////////
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&WaypointActionClass::execute, this, _1), goal_handle}
      .detach();
}

void WaypointActionClass::execute(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

  ///////////////////////////////
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  rclcpp::Rate _rate(25);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<WaypointAction::Feedback>();
  auto result = std::make_shared<WaypointAction::Result>();

  // define goal and errors
  this->_des_pos = goal->position;
  double desired_yaw = atan2(this->_des_pos.y - this->_position.y,
                             this->_des_pos.x - this->_position.x);
  double err_pos = sqrt(pow(this->_des_pos.y - this->_position.y, 2) +
                        pow(this->_des_pos.x - this->_position.x, 2));
  double err_yaw = desired_yaw - this->_yaw;

  // loop for execution
  while (err_pos > this->_dist_precision && rclcpp::ok()) {
    // update vars
    desired_yaw = atan2(this->_des_pos.y - this->_position.y,
                             this->_des_pos.x - this->_position.x);
    err_pos = sqrt(pow(this->_des_pos.y - this->_position.y, 2) +
                        pow(this->_des_pos.x - this->_position.x, 2));
    err_yaw = desired_yaw - this->_yaw;
    
    RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", this->_yaw);
    RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", desired_yaw);
    RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", err_yaw);
    // logic goes here
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "The goal has been cancelled/preempted");
      result->success = false;
      goal_handle->canceled(result);
      return;

    } else if (std::abs(err_yaw) > this->_yaw_precision) {
      RCLCPP_INFO(this->get_logger(), "fixed yaw");
      this->_state = "fix yaw";
      geometry_msgs::msg::Twist vel_msg;
      
      if (err_yaw > 0) {
        vel_msg.angular.z = 0.65;
      } else {
        vel_msg.angular.z = -0.65;
      }
      this->vel_pub_->publish(vel_msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "go to point");
      this->_state = "go to point";
      geometry_msgs::msg::Twist vel_msg;
      vel_msg.linear.x = 0.6;
      vel_msg.angular.z = 0;
      this->vel_pub_->publish(vel_msg);
    }

    feedback->position = this->_position;
    feedback->state = this->_state;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");

    _rate.sleep();
  }
  // stop robot if goal reached
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x = 0;
  vel_msg.angular.z = 0;
  this->vel_pub_->publish(vel_msg);

  // return result
  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    std::shared_ptr<WaypointActionClass> node = std::make_shared<WaypointActionClass>();

    rclcpp::executors::MultiThreadedExecutor srv_exe;
    srv_exe.add_node(node);
    srv_exe.spin();

    rclcpp::shutdown();
    return 0;
}
