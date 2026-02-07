#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class EightTrajectory : public rclcpp::Node {
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;
  using Odometry = nav_msgs::msg::Odometry;

public:
  EightTrajectory() : Node("eight_trajectory") {
    pub_ = create_publisher<Float32MultiArray>("/wheel_speed", 10);
    // while (pub_->get_subscription_count() == 0 && rclcpp::ok()) {
    //   RCLCPP_INFO(get_logger(), "Waiting for /wheel_speed subscriber");
    //   rclcpp::sleep_for(std::chrono::milliseconds(200));
    // }
    sub_ = create_subscription<Odometry>(
        "/rosbot_xl_base_controller/odom", 10,
        [this](Odometry::SharedPtr msg) { odom_callback(msg); });

    h_wheel_base_ = 0.170 / 2.0;
    h_track_width_ = 0.26969 / 2.0;
    radius_ = 0.100 / 2.0;
    RCLCPP_INFO(get_logger(), "Initialized node");
  }

private:
  void odom_callback(const Odometry::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "msg is received");
    auto &orientation = msg->pose.pose.orientation;
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z,
                         orientation.w);
    tf2::Matrix3x3(quat).getRPY(_, _, yaw_);
  }
  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  float h_wheel_base_;
  float h_track_width_;
  float radius_;
  float yaw_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}