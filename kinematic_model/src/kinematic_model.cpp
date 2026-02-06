#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class KinematicModel : public rclcpp::Node {
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;
  using Twist = geometry_msgs::msg::Twist;

public:
  KinematicModel() : Node("kinematic_model") {
    sub_ = create_subscription<Float32MultiArray>(
        "/wheel_speed", 10,
        [this](Float32MultiArray::SharedPtr msg) { callback(msg); });
    pub_ = create_publisher<Twist>("/cmd_vel", 10);
    RCLCPP_INFO(get_logger(), "Node initialized");
  }

private:
  void callback(const Float32MultiArray::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "msg is received");
  }
  rclcpp::Subscription<Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}