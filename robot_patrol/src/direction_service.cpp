#include "direction_srv/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <string>
#include <unistd.h>

using GetDirection = direction_srv::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("get_direction_server") {
    srv_ = create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::turn_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;
  float total_dist_sec_right;
  float total_dist_sec_front;
  float total_dist_sec_left;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  void turn_callback(const std::shared_ptr<GetDirection::Request> request,
                     const std::shared_ptr<GetDirection::Response> response) {
    // Use the data received in the request part of the message to make the
    // robot GetDirection:
    RCLCPP_INFO(this->get_logger(), "Processing Request");
    string result = "front";
    for (int i = 180; i < 300; i++) {
      total_dist_sec_right =
          total_dist_sec_right + request->laser_data.ranges[i];
    }
    for (int j = 300; j < 420; j++) {
      total_dist_sec_front =
          total_dist_sec_front + request->laser_data.ranges[j];
    }
    for (int k = 420; k < 540; k++) {
      total_dist_sec_left = total_dist_sec_left + request->laser_data.ranges[k];
    }
    RCLCPP_INFO(this->get_logger(), "Right is %f", total_dist_sec_right);
    RCLCPP_INFO(this->get_logger(), "Front is %f", total_dist_sec_front);
    RCLCPP_INFO(this->get_logger(), "Left is %f", total_dist_sec_left);
    if (total_dist_sec_front < total_dist_sec_left) {
      result = "left";
    }
    if (total_dist_sec_front < total_dist_sec_right && total_dist_sec_left < total_dist_sec_right) {
      result = "right";
    }
    RCLCPP_INFO(this->get_logger(), "Result is %s", result.c_str());
    response->direction = result;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
