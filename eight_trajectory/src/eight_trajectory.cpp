#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

/*
 * Generates a figure-eight trajectory using incremental waypoints.
 * Converts pose error -> body twist -> wheel angular velocities for
 * a 4-wheel omnidirectional robot.
 *
 * Wheel order:
 *   u1 = Front Left
 *   u2 = Front Right
 *   u3 = Rear Right
 *   u4 = Rear Left
 *
 * Kinematic chain:
 *   pose error (world frame) [yaw_err, x_err, y_err]
 *        ↓
 *   body twist [wz, vx, vy]
 *        ↓
 *   wheel angular velocities [u1, u2, u3, u4]
 */
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

    H_ = compute_H_matrix();
    yaw_ = std::numeric_limits<float>::quiet_NaN();
    x_ = std::numeric_limits<float>::quiet_NaN();
    y_ = std::numeric_limits<float>::quiet_NaN();
    target_yaw_ = 0.0;
    target_x_ = 0.0;
    target_y_ = 0.0;

    //  Waypoints are relative increments: [dyaw, dx, dy]
    //  Applied cumulatively to generate a figure-eight motion.
    way_points_ = {
        {0.0, 0.0, 0.0},  {0.0, 1.0, -1.0},     {0.0, 1.0, 1.0},
        {0.0, 1.0, 1.0},  {-1.5708, 1.0, -1.0}, {-1.5708, -1.0, -1.0},
        {0.0, -1.0, 1.0}, {0.0, -1.0, 1.0},     {0.0, -1.0, -1.0}};
    way_point_count_ = 0;

    RCLCPP_INFO(get_logger(), "Initialized node");
  }

private:
  void odom_callback(const Odometry::SharedPtr msg) {
    auto &orientation = msg->pose.pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z,
                      orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_ = yaw;

    auto &position = msg->pose.pose.position;
    x_ = position.x;
    y_ = position.y;
  }

  void motion_callback() {
    if (std::isnan(yaw_) || std::isnan(x_) || std::isnan(y_)) {
      return;
    }
    if (is_reached()) {
      update_way_point_count();
    }

    auto twist = compute_body_twist();
    auto wheels = compute_wheel_speeds(twist);
    publish_velocity(wheels);
  }

  std::tuple<float, float, float> get_errors() const {
    float error_yaw = normalize_angle(target_yaw_ - yaw_);
    float error_x = target_x_ - x_;
    float error_y = target_y_ - y_;
    return {error_yaw, error_x, error_y};
  }

  float normalize_angle(float angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  bool is_reached() const {
    auto [err_yaw, err_x, err_y] = get_errors();
    return std::abs(err_yaw) < 0.1 && std::hypot(err_x, err_y) < 0.05;
  }

  void update_way_point_count() {
    way_point_count_++;
    if (way_point_count_ >= way_points_.size()) {
      publish_velocity({0.0, 0.0, 0.0, 0.0});
      RCLCPP_INFO(get_logger(), "Stop robot");
      timer_->cancel();
      return;
    }
    auto wp = way_points_[way_point_count_];
    update_target(wp[0], wp[1], wp[2]);
  }

  void update_target(float yaw, float x, float y) {
    target_yaw_ += yaw;
    target_x_ += x;
    target_y_ += y;
    RCLCPP_INFO(get_logger(), "Moving to the waypoint %zu, [%f, %f, %f]",
                way_point_count_, target_yaw_, target_x_, target_y_);
  }

  /*
   * Convert pose error (world frame) -> body frame twist
   * Let:
   *   θ      = current robot yaw (world frame)
   *   eθ     = yaw error = normalize(target_yaw − yaw)
   *   ex, ey = position error in world frame
   *
   * Convert translational error from world frame → body frame:
   *   vx =  cos(θ) * ex + sin(θ) * ey
   *   vy = -sin(θ) * ex + cos(θ) * ey
   * Angular velocity command:
   *   wz = eθ
   *
   * Result:
   *   body_twist = [wz, vx, vy]
   */
  std::vector<float> compute_body_twist() const {
    auto [dphi, dx, dy] = get_errors();
    float vx_body = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    float vy_body = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;

    return {dphi, vx_body, vy_body};
  }

  /*
   * Maps body twist -> wheel angular velocities using mecanum model
   * Robot geometry:
   *   L = half wheel base
   *   W = half track width
   *   r = wheel radius
   *
   * Define k = (L + W) / r
   *
   * Wheel speed mapping (mecanum drive):
   *   u1 = (-k * wz + vx - vy)
   *   u2 = ( k * wz + vx + vy)
   *   u3 = ( k * wz + vx - vy)
   *   u4 = (-k * wz + vx + vy)
   *
   * Final output:
   *   wheel_speeds = [u1, u2, u3, u4]
   */
  std::vector<float>
  compute_wheel_speeds(const std::vector<float> &twist) const {
    std::vector<float> wheels(4, 0.0);
    for (size_t i = 0; i < H_.size(); i++) {
      wheels[i] =
          H_[i][0] * twist[0] + H_[i][1] * twist[1] + H_[i][2] * twist[2];
    }
    return wheels;
  }

  // Build wheel mapping matrix H
  //    Input : body twist [wz, vx, vy]
  //    Output: wheel speeds [u1, u2, u3, u4]
  //
  // Geometry:
  //  L = half wheel base
  //  W = half track width
  //  r = wheel radius
  std::vector<std::vector<float>> compute_H_matrix() const {
    float L = 0.170 / 2.0;
    float W = 0.26969 / 2.0;
    float r = 0.100 / 2.0;
    std::vector<std::vector<float>> H = {{-(L + W), 1.0, -1.0},
                                         {(L + W), 1.0, 1.0},
                                         {(L + W), 1.0, -1.0},
                                         {-(L + W), 1.0, 1.0}};
    for (auto &row : H) {
      for (auto &col : row) {
        col /= r;
      }
    }
    return H;
  }

  void publish_velocity(const std::vector<float> &velocities) const {
    if (velocities.size() != 4) {
      RCLCPP_WARN(get_logger(), "Expects 4 wheel speeds, received %zu",
                  velocities.size());
      return;
    }

    Float32MultiArray msg;
    msg.data = velocities;
    pub_->publish(msg);
  }

  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::vector<float>> way_points_;
  size_t way_point_count_;
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