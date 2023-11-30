#include "robot_patrol/srv/GetDirection.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <unistd.h>
using GetDirection = robot_patrol::srv::direction_service;
using std::placeholders::_1;
using std::placeholders::_2;
class DirectionService : public rclcpp::Node {
public:
  ServerNode() : Node("get_direction_server") {
    srv_ = create_service<Spin>(
        "/direction_service",
        std::bind(&ServerNode::turn_callback, this, _1, _2));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  float total_dist_sec_right;
  float total_dist_sec_front;
  float total_dist_sec_left;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  void turn_callback(const std::shared_ptr<Spin::Request> request,
                     const std::shared_ptr<Spin::Response> response, ) {
    // Use the data received in the request part of the message to make the
    // robot spin:
    string result_dir = "front";
    for (int i = 180; i < 300; i++) {
      total_dist_sec_right = total_dist_sec_right + request->ranges[i];
    }
    for (int i = 300; i < 480; i++) {
    total_dist_sec_front = total_dist_sec_front + request->ranges[i];
    }
    for (int i = 480; i < 540; i++) {
    total_dist_sec_left = total_dist_sec_left + request->ranges[i];
    }
    if (total_dist_sec_front < total_dist_sec_left){
        result = "left";
    }
    if (total_dist_sec_front < total_dist_sec_right){
        result = "right";    
    }
    requst->direction = result;

  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServerNode>());
    rclcpp::shutdown();
    return 0;
  }
