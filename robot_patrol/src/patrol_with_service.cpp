#include "direction_srv/srv/get_direction.hpp"
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
using namespace std;

class Patrol : public rclcpp::Node {
private:
  rclcpp::CallbackGroup::SharedPtr sub_callback_group;
  rclcpp::CallbackGroup::SharedPtr pub_callback_group;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
  string result;
  sensor_msgs::msg::LaserScan last_laser_;
  geometry_msgs::msg::Twist move_vector;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<direction_srv::srv::GetDirection>::SharedPtr client_;
  bool laser_received = false;
  bool service_done_ = false;
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Front laser %f", msg->ranges[360]);
    last_laser_ = *msg;
    laser_received = true;
  }

  void move() {
    if (laser_received) {
      auto request =
          std::make_shared<direction_srv::srv::GetDirection::Request>();
      request->laser_data = last_laser_;
    
      auto result_future = client_->async_send_request(
          request,
          std::bind(&Patrol::response_callback, this, std::placeholders::_1));

      if (result == "front") {
        move_vector.linear.x = 0.1;
        move_vector.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "moving forward");
      }
      if (result == "left") {
        move_vector.linear.x = 0.1;
        move_vector.angular.z = 0.5;
        RCLCPP_INFO(this->get_logger(), "moving left");
      }
      if (result == "right") {
        move_vector.linear.x = 0.1;
        move_vector.angular.z = -0.5;
        RCLCPP_INFO(this->get_logger(), "moving right");
      }
      RCLCPP_INFO(this->get_logger(), "publishing move_vector");
      publisher_->publish(move_vector);
    } else {
      RCLCPP_INFO(this->get_logger(), "waiting for laser");
    }
  }

  void response_callback(
      rclcpp::Client<direction_srv::srv::GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Result: success");
      result = future.get()->direction;
      RCLCPP_INFO(this->get_logger(), result);
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

public:
  Patrol() : Node("patrol_city") {
    pub_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_ = this->create_client<direction_srv::srv::GetDirection>(
        "/direction_service");
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