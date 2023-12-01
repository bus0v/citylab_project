
#include "direction_srv/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
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

    string result = "front";
    for (int i = 180; i < 300; i++) {
      total_dist_sec_right = total_dist_sec_right + request->laser_data.ranges[i];
    }
    for (int i = 300; i < 480; i++) {
      total_dist_sec_front = total_dist_sec_front + request->laser_data.ranges[i];
    }
    for (int i = 480; i < 540; i++) {
      total_dist_sec_left = total_dist_sec_left + request->laser_data.ranges[i];
    }
    if (total_dist_sec_front < total_dist_sec_left) {
      result = "left";
    }
    if (total_dist_sec_front < total_dist_sec_right) {
      result = "right";
    }
    response->direction = result;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
