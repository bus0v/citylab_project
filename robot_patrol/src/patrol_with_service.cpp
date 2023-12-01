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

  geometry_msgs::msg::Twist move_vector;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
      node->create_client<std_srvs::srv::Empty>("/direction_service");

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Front laser %f", msg->ranges[360]);
    float max = 0.0;
    int max_pos;
    auto request = msg;

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    auto result_future = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
      move(result)
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service /moving");
    }
  }

  void move(string result) {
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