#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cmath>
#include <memory>
using namespace std::chrono_literals;
using std::placeholders::_1;
class Patrol : public rclcpp::Node {
private:
  rclcpp::CallbackGroup::SharedPtr sub_callback_group;
  rclcpp::CallbackGroup::SharedPtr pub_callback_group;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
  float direction_;
  bool frontBlocked;
  geometry_msgs::msg::Twist move_vector;
  rclcpp::TimerBase::SharedPtr timer_;
 
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Front laser %f", msg->ranges[360]);
    float max = 0.0;
    int max_pos;

    for (int i = 180; i < 540; i++) {
      if (msg->ranges[i] > max && msg->ranges[i] != INFINITY) {
        max = msg->ranges[i];
        max_pos = i - 380;
      }
      if (msg->ranges[360] < 0.87) {
        frontBlocked = true;
      } else {
        frontBlocked = false;
      }
    }

    direction_ = max_pos * msg->angle_increment;
    // RCLCPP_INFO(this->get_logger(), "angle increment  %f",
    // msg->angle_increment); RCLCPP_INFO(this->get_logger(), "ranges length
    // %i", msg->ranges.size()); RCLCPP_INFO(this->get_logger(), "min angle %f",
    // msg->angle_min); RCLCPP_INFO(this->get_logger(), "max pos  %i", max_pos);

    RCLCPP_INFO(this->get_logger(), "Safest distance is  %f at angle %f", max,
                direction_);
  }

  void move() {
    if (frontBlocked) {
      move_vector.linear.x = 0.1;
      move_vector.angular.z = direction_ / 2;
      RCLCPP_INFO(this->get_logger(), "angular speed is  %f", direction_ / 2);
    } else {
      move_vector.linear.x = 0.1;
      move_vector.angular.z = 0.0;
    }
    publisher_->publish(move_vector);
  }

public:
  Patrol() : Node("patrol_city") {
    pub_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options_sub;
    options_sub.callback_group = sub_callback_group;
    
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::laser_callback, this, _1), options_sub);

    timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::move, this),
                                     pub_callback_group);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}