#include <cmath>
#include <functional>
#include <math.h>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

class GoToPose : public rclcpp::Node {
public:
  using GoTo = robot_patrol::action::GoToPose;
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
    // RCLCPP_INFO(this->get_logger(), "Current yaw is %f", atan2(t3, t4));
  }

  void execute(const std::shared_ptr<GoalHandleGoTo> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoTo::Feedback>();
    auto result = std::make_shared<GoTo::Result>();
    auto move = geometry_msgs::msg::Twist();
    
    rclcpp::Rate loop_rate(1);
    float dx, dy, dtheta, h;
    dx = goal->goal_pos.x - current_pos_.x;
    dy = goal->goal_pos.y - current_pos_.y;
    dtheta = goal->goal_pos.theta * 0.0174533 - current_pos_.theta;
    h = sqrt(pow(dx, 2) + pow(dy, 2));
    float angle = acos(dx / h);
    bool turned = false;
    RCLCPP_INFO(this->get_logger(), "dx = %f", dx);
    RCLCPP_INFO(this->get_logger(), "dy = %f", dy);
    RCLCPP_INFO(this->get_logger(), "h = %f", dx);
    RCLCPP_INFO(this->get_logger(), "angle = %f", angle * 57.2958);
    float angle_diff;
    while (!result->status && rclcpp::ok()) {

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      dx = abs(goal->goal_pos.x - current_pos_.x);
      dy = abs(goal->goal_pos.y - current_pos_.y);
      
      dtheta = goal->goal_pos.theta * 0.0174533 - current_pos_.theta;
     
      if (!turned){
       RCLCPP_INFO(this->get_logger(), "angle to turn = %f", angle);
       RCLCPP_INFO(this->get_logger(), "current angle = %f", current_pos_.theta);
        angle_diff = (angle - current_pos_.theta);
        RCLCPP_INFO(this->get_logger(), "angle diff= %f", abs(angle_diff));
        float turn_speed = 0.3;
        if (angle_diff < 0) {turn_speed = -0.3;}
        move.angular.z = angle_diff/2;
        move.linear.x = 0.0;
        if (abs(angle_diff) < 0.012){turned = true;}
      }
      // turn the robot until the differce between the angle heading and the curent theta is 0.01
      //mark the turn as completed
      
      // Move robot forward and send feedback
      if (turned){
      RCLCPP_INFO(this->get_logger(), "dx = %f", dx);
      RCLCPP_INFO(this->get_logger(), "dy = %f", dy);
        move.linear.x = 0.1;
        move.angular.z = 0.0;
      }
      
      if (dx < 0.1 && dy < 0.1) {
        move.linear.x = 0.0;
        move.angular.z = dtheta / 2;
        RCLCPP_INFO(this->get_logger(), "dtheta = %f", dtheta);
      }

      if (dx < 0.1 && dy < 0.1 && abs(dtheta) < 0.1) {
        result->status = true;
      }

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