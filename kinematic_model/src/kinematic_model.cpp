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

    h_wheel_base_ = 0.170 / 2.0;
    h_track_width_ = 0.26969 / 2.0;
    radius_ = 0.100 / 2.0;
    RCLCPP_INFO(get_logger(), "Node initialized");
  }

private:
  void callback(const Float32MultiArray::SharedPtr msg) {
    auto u_1 = msg->data[0];
    auto u_2 = msg->data[1];
    auto u_3 = msg->data[2];
    auto u_4 = msg->data[3];

    float vx = radius_ / 4.0 * (u_1 + u_2 + u_3 + u_4);
    float vy = radius_ / 4.0 * (-u_1 + u_2 - u_3 + u_4);
    float vw = radius_ / (4.0 * (h_wheel_base_ + h_track_width_)) *
               (-u_1 + u_2 + u_3 - u_4);

    Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vw;
    pub_->publish(cmd_vel);
  }
  rclcpp::Subscription<Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  float h_wheel_base_;
  float h_track_width_;
  float radius_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}