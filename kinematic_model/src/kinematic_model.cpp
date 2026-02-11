#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

/*
 * Converts individual wheel angular velocities into robot body velocity
 * (vx, vy, vw) for a 4-wheel omnidirectional drive robot.
 *
 * Wheel convention used:
 *   u1 = Front Left wheel
 *   u2 = Front Right wheel
 *   u3 = Rear Right wheel
 *   u4 = Rear Left wheel
 *
 * Kinematic model (mecanum inverse kinematics):
 *
 *   vx = (r/4) * ( u1 + u2 + u3 + u4 )
 *   vy = (r/4) * (-u1 + u2 - u3 + u4 )
 *   vw = (r / (4*(L + W))) * (-u1 + u2 + u3 - u4)
 *
 * where:
 *   r = wheel radius
 *   L = half of wheel base (front-rear distance / 2)
 *   W = half of track width (left-right distance / 2)
 *
 */
class KinematicModel : public rclcpp::Node {
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;
  using Twist = geometry_msgs::msg::Twist;

public:
  KinematicModel() : Node("kinematic_model") {
    sub_ = create_subscription<Float32MultiArray>(
        "/wheel_speed", 10,
        [this](Float32MultiArray::SharedPtr msg) { callback(msg); });
    pub_ = create_publisher<Twist>("/cmd_vel", 10);

    // Robot geometric parameters (meters)
    h_wheel_base_ = 0.170 / 2.0;    // L
    h_track_width_ = 0.26969 / 2.0; // W
    radius_ = 0.100 / 2.0;          // r
    RCLCPP_INFO(get_logger(), "Initialized node");
  }

private:
  void callback(const Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(get_logger(), "Expects 4 wheel speeds, received %zu",
                  msg->data.size());
      return;
    }

    auto [vx, vy, vw] =
        get_velocity(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    publish_velocity(vx, vy, vw);
  }

  std::tuple<float, float, float> get_velocity(float u1, float u2, float u3,
                                               float u4) const {
    // Convert wheel angular velocity (rad/s) to robot body velocity
    // (m/s,m/s,rad/s)
    float vx = radius_ / 4.0 * (u1 + u2 + u3 + u4);
    float vy = radius_ / 4.0 * (-u1 + u2 - u3 + u4);
    float vw = radius_ / (4.0 * (h_wheel_base_ + h_track_width_)) *
               (-u1 + u2 + u3 - u4);
    return {vx, vy, vw};
  }

  void publish_velocity(float vx, float vy, float vw) const {
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