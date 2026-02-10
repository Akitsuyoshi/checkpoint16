#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class EightTrajectory : public rclcpp::Node {
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;
  using Odometry = nav_msgs::msg::Odometry;

public:
  EightTrajectory() : Node("eight_trajectory") {
    pub_ = create_publisher<Float32MultiArray>("/wheel_speed", 10);
    while (pub_->get_subscription_count() == 0 && rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "Waiting for /wheel_speed subscriber");
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    sub_ = create_subscription<Odometry>(
        "/odometry/filtered", 10,
        [this](Odometry::SharedPtr msg) { odom_callback(msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               [this]() { motion_callback(); });

    H_ = get_H_();
    x_ = std::numeric_limits<float>::quiet_NaN();
    y_ = std::numeric_limits<float>::quiet_NaN();
    yaw_ = std::numeric_limits<float>::quiet_NaN();
    way_points_ = {{0.0, 1.0, -1.0},      {0.0, 1.0, 1.0},
                   {0.0, 1.0, 1.0},       {-1.5708, 1.0, -1.0},
                   {-1.5708, -1.0, -1.0}, {0.0, -1.0, 1.0},
                   {0.0, -1.0, 1.0},      {0.0, -1.0, -1.0}};
    way_point_count_ = 0;
    target_yaw_ = way_points_[way_point_count_][0];
    target_x_ = way_points_[way_point_count_][1];
    target_y_ = way_points_[way_point_count_][2];
    RCLCPP_INFO(get_logger(), "Initialized node");
    RCLCPP_INFO(get_logger(), "Moving to the waypoint %zu, [%f, %f, %f]",
                way_point_count_ + 1, target_yaw_, target_x_, target_y_);
  }

private:
  void odom_callback(const Odometry::SharedPtr msg) {
    auto &position = msg->pose.pose.position;
    x_ = position.x;
    y_ = position.y;

    auto &orientation = msg->pose.pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z,
                      orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_ = yaw;
  }

  void motion_callback() {
    if (std::isnan(yaw_)) {
      return;
    }

    std::vector<float> errors = get_error();
    bool is_reached =
        std::hypot(errors[1], errors[2]) < 0.05 && std::abs(errors[0]) < 0.1;
    if (is_reached) {
      way_point_count_++;
      if (way_point_count_ >= way_points_.size()) {
        timer_->cancel();
        Float32MultiArray msg;
        msg.data = {0, 0, 0, 0};
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Stop robot");
        return;
      }

      auto way_point = way_points_[way_point_count_];
      target_yaw_ += way_point[0];
      target_x_ += way_point[1];
      target_y_ += way_point[2];
      RCLCPP_INFO(get_logger(), "Moving to the waypoint %zu, [%f, %f, %f]",
                  way_point_count_ + 1, target_yaw_, target_x_, target_y_);
    }

    auto way_point = way_points_[way_point_count_];
    errors = get_error();
    float K = 0.9;
    float dphi = K * errors[0];
    float dx = K * errors[1];
    float dy = K * errors[2];

    Float32MultiArray msg;
    auto twist = velocity_to_twist(dphi, dx, dy);
    msg.data = twist_to_wheels(twist[0], twist[1], twist[2]);
    pub_->publish(msg);
  }

  std::vector<float> get_error() const {
    float error_yaw = normalize_angle(target_yaw_ - yaw_);
    float error_x = target_x_ - x_;
    float error_y = target_y_ - y_;
    return {error_yaw, error_x, error_y};
  }

  float normalize_angle(float angle) const {
    return std::atan2(sin(angle), cos(angle));
  }

  std::vector<float> velocity_to_twist(float dphi, float dx, float dy) const {
    std::vector<std::vector<float>> R = {
        {1.0, 0.0, 0.0},
        {0.0, std::cos(yaw_), std::sin(yaw_)},
        {0.0, -std::sin(yaw_), std::cos(yaw_)}};
    std::vector<float> vels = {dphi, dx, dy};

    std::vector<float> twist;
    for (size_t i = 0; i < R.size(); i++) {
      float temp = 0;
      for (size_t j = 0; j < R[i].size(); j++) {
        temp += R[i][j] * vels[j];
      }
      twist.emplace_back(temp);
    }

    return twist;
  }

  std::vector<float> twist_to_wheels(float wz, float vx, float vy) const {
    std::vector<float> vels = {wz, vx, vy};
    std::vector<float> wheel_vels;
    for (size_t i = 0; i < H_.size(); i++) {
      float temp = 0;
      for (size_t j = 0; j < H_[i].size(); j++) {
        temp += H_[i][j] * vels[j];
      }
      wheel_vels.emplace_back(temp);
    }

    return wheel_vels;
  }

  std::vector<std::vector<float>> get_H_() const {
    float h_wheel_base_ = 0.170 / 2.0;
    float h_track_width_ = 0.26969 / 2.0;
    float radius_ = 0.100 / 2.0;
    std::vector<std::vector<float>> H = {
        {-(h_wheel_base_ + h_track_width_), 1.0, -1.0},
        {(h_wheel_base_ + h_track_width_), 1.0, 1.0},
        {(h_wheel_base_ + h_track_width_), 1.0, -1.0},
        {-(h_wheel_base_ + h_track_width_), 1.0, 1.0}};

    for (auto &row : H) {
      for (auto &col : row) {
        col /= radius_;
      }
    }

    return H;
  }

  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::vector<float>> way_points_;
  size_t way_point_count_;
  std::vector<float> wheel_vels_;
  std::vector<std::vector<float>> H_;
  float yaw_;
  float x_;
  float y_;
  float target_yaw_;
  float target_x_;
  float target_y_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}