#include <cmath>
#include <functional>
#include <math.h>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "gotopose_act/action/go_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GoToPose : public rclcpp::Node {
public:
  using GoTo = gotopose_act::action::GoToPose;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;
  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoTo>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<GoTo>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoTo::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with pose %f,%f,%f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoTo> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoTo> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.set__x(msg->pose.pose.position.x);
    current_pos_.set__y(msg->pose.pose.position.y);
    // current_pos_.set__theta(msg->pose.pose.orientation.theta);
    float w, z, x, y, t3, t4;
    x = msg->pose.pose.orientation.x;
    y = msg->pose.pose.orientation.y;
    z = msg->pose.pose.orientation.z;
    w = msg->pose.pose.orientation.w;

    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (y * y + z * z);
    current_pos_.set__theta(atan2(t3, t4));
    //RCLCPP_INFO(this->get_logger(), "Current yaw is %f", atan2(t3, t4));
  }

  void execute(const std::shared_ptr<GoalHandleGoTo> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoTo::Feedback>();
    auto result = std::make_shared<GoTo::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    while (!result->status && rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      float dx, dy, dtheta, h;
      dx = goal->goal_pos.x - current_pos_.x;
      dy = goal->goal_pos.y - current_pos_.y;
      dtheta = goal->goal_pos.theta - current_pos_.theta;
      h = sqrt(pow(dx,2) + pow(dy,2));
      float t = h / 0.2;
      RCLCPP_INFO(this->get_logger(), "h = %f",h);
      RCLCPP_INFO(this->get_logger(), "theta_speed = %f",dtheta/t);
      if (dx < 0.1 && dy < 0.1 && dtheta < 0.1) {
        result->status = true;
      }
      // Move robot forward and send feedback
      move.linear.x = 0.2;
      move.angular.z = dtheta/t;
      publisher_->publish(move);
      feedback->current_pos = this->current_pos_;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = "Finished action server. Robot moved during 5 seconds";
      move.linear.x = 0.0;
      move.angular.z = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}