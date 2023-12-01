#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tortoisebot_waypoints_interface/action/detail/waypoint_action__struct.hpp"
#include "tortoisebot_waypoints_interface/action/waypoint_action.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>

using Point = geometry_msgs::msg::Point;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointActionClient : public rclcpp::Node {

public:
  using WaypointAction =
      tortoisebot_waypoints_interface::action::WaypointAction;
  using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<WaypointAction>;

  explicit WaypointActionClient(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("waypoint_action_client", options) {

    this->client_ptr_ =
        rclcpp_action::create_client<WaypointAction>(this, "tortoisebot_as");
  }

  bool is_goal_done() { return this->goal_done_; }

  void send_goal(Point des_pos_) {
    using namespace std::placeholders;

    this->goal_done_ = false;
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      rclcpp::shutdown();
    }

    auto goal_msg = WaypointAction::Goal();
    goal_msg.position = des_pos_;
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WaypointActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&WaypointActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&WaypointActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  // cancel goal
  void cancel_goal(){
    
    this->client_ptr_->async_cancel_all_goals();
  }


private:
  void goal_response_callback(
      std::shared_future<GoalHandleWaypoint::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleWaypoint::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "current position: [x: %f, y: %f, z: %f]",
                feedback->position.x, feedback->position.y,
                feedback->position.z);
    RCLCPP_INFO(this->get_logger(), "current state: %s",
                feedback->state.c_str());
  }

  void result_callback(const GoalHandleWaypoint::WrappedResult &result) {

    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "result: %s",
                (result.result->success) ? "true" : "false");
  }

  rclcpp_action::Client<WaypointAction>::SharedPtr client_ptr_;
  bool goal_done_;
  
};

///////////////////////////////////////////////////////
class WaypointActionServerTest : public testing::Test {

  // create client instance
public:
  using Odometry = nav_msgs::msg::Odometry;
  WaypointActionServerTest() {

    this->action_client_ = std::make_shared<WaypointActionClient>();
    this->odom_sub_ = action_client_->create_subscription<Odometry>(
        "odom", 10,
        std::bind(&WaypointActionServerTest::odom_callback, this,
                  std::placeholders::_1));
  }

  void err_test(const Point &des_pos_);
  void cancel_goal_test(const Point &des_pos_);

private:
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<WaypointActionClient> action_client_;
  Point position_;
  double yaw_;

  void odom_callback(const Odometry::SharedPtr msg) {
    this->position_ = msg->pose.pose.position;

    // yaw
    auto quaternion = tf2::Quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    // convert to RPY
    tf2::Matrix3x3 m(quaternion);
    double r, p;
    m.getRPY(r, p, this->yaw_);
  }
};

void WaypointActionServerTest::err_test(const Point &des_pos_) {

  action_client_->send_goal(des_pos_);

  while (!action_client_->is_goal_done()) {
    rclcpp::spin_some(action_client_);
  }

  double err_pos = sqrt(pow(des_pos_.y - this->position_.y, 2) +
                        pow(des_pos_.x - this->position_.x, 2));
  
  double des_yaw_ =
      atan2(des_pos_.y - this->position_.y, des_pos_.x - this->position_.x);
  double err_yaw = des_yaw_ - this->yaw_;

  EXPECT_LE(err_pos, 0.1);
  EXPECT_LE(std::abs(err_yaw), 0.5); // +- 2 degree
}

void WaypointActionServerTest::cancel_goal_test(const Point &des_pos_) {

  action_client_->send_goal(des_pos_);

  while (!action_client_->is_goal_done()) {
    rclcpp::spin_some(action_client_);
    this->action_client_->cancel_goal();
    break;
  }
  ASSERT_TRUE(false);

}

TEST_F(WaypointActionServerTest, ErrorTest) {

  // goal position
  Point des_pos_;
  des_pos_.x = 0.5;
  des_pos_.y = -0.5;
  err_test(des_pos_);
}

TEST_F(WaypointActionServerTest, CancelGoal) {
  // goal position
  Point des_pos_;
  des_pos_.x = 0.7;
  des_pos_.y = -0.7;
  cancel_goal_test(des_pos_);
}


