#include "direction_srv/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
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
    total_dist_sec_right = 0;
    total_dist_sec_front = 0;
    total_dist_sec_left = 0;

    RCLCPP_INFO(this->get_logger(), "Processing Request");
    string result = "front";
    RCLCPP_INFO(this->get_logger(), "ranges data %f",request->laser_data.ranges[0]);
    for (int i = 180; i < 300; i++) {
      total_dist_sec_right =
          total_dist_sec_right + request->laser_data.ranges[i];
          //RCLCPP_INFO(this->get_logger(), "Added right ranges");
    }
    for (int j = 300; j < 420; j++) {
      total_dist_sec_front =
          total_dist_sec_front + request->laser_data.ranges[j];
          //(this->get_logger(), "Added front ranges");
    }
    for (int k = 420; k < 540; k++) {
      total_dist_sec_left = total_dist_sec_left + request->laser_data.ranges[k];
      //RCLCPP_INFO(this->get_logger(), "Added left ranges");
    }
    RCLCPP_INFO(this->get_logger(), "Front total %f", total_dist_sec_front);
    RCLCPP_INFO(this->get_logger(), "Left total %f", total_dist_sec_left);
    RCLCPP_INFO(this->get_logger(), "Right total %f", total_dist_sec_right);
    if (total_dist_sec_front < total_dist_sec_left && total_dist_sec_front < 100.0) {
      result = "left";
    }
    if (total_dist_sec_front < total_dist_sec_right && total_dist_sec_left < total_dist_sec_right && total_dist_sec_front < 100.0) {
      result = "right";
    }
    RCLCPP_INFO(this->get_logger(), "Result is %s", result.c_str());
    response->direction = result;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DirectionService> direction_node = std::make_shared<DirectionService>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(direction_node);
  executor.spin();
  RCLCPP_INFO(direction_node->get_logger(), "shutting down");
  //rclcpp::shutdown();
  return 0;
}
