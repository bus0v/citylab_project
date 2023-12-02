#include "direction_srv/srv/detail/get_direction__builder.hpp"
#include "direction_srv/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class ServiceClient : public rclcpp::Node {
private:
  rclcpp::Client<direction_srv::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
  bool service_done_ = false;
  string result;

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

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto request =
        std::make_shared<direction_srv::srv::GetDirection::Request>();
    request->laser_data = *msg;
    service_done_ = false;
    RCLCPP_INFO(this->get_logger(), "Sending Request");
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Direction recieved %s", result.c_str());
  }

public:
  ServiceClient() : Node("service_client") {
    client_ = this->create_client<direction_srv::srv::GetDirection>(
        "/direction_service");
    laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ServiceClient::laser_callback, this, _1));
  }

  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<ServiceClient>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}