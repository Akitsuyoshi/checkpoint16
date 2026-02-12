#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

/*
 * 4-wheel omnidirectional robot model.
 *
 * Wheel order in published /wheel_speed message:
 *   [0] Front Left (FL)
 *   [1] Front Right (FR)
 *   [2] Rear Left (RL)
 *   [3] Rear Right (RR)
 *
 * Robot body frame:
 *   +X : forward
 *   +Y : left
 *   +Yaw : counter-clockwise rotation
 *
 * The motion patterns below are simplified open-loop wheel velocity commands.
 * Full inverse kinematics (wheel radius, robot geometry, scaling) is NOT used.
 *
 * Motion timing:
 *   Each motion lasts 3 s
 *   0–2 s : constant wheel speed
 *   2–3 s : linear ramp-down to zero
 */
class WheelVelocitiesPublisher : public rclcpp::Node {
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;

  enum class Motion {
    Forward,
    Backward,
    Left,
    Right,
    TurnClock,
    TurnCounterClock,
    Stop,
  };

public:
  WheelVelocitiesPublisher()
      : Node("wheel_velocities_publisher"), motion_count_(0),
        target_vels_({0.0, 0.0, 0.0, 0.0}), output_vels_({0.0, 0.0, 0.0, 0.0}),
        motion_duration_(3.0), ramp_start_time_(2.0) {
    pub_ = create_publisher<Float32MultiArray>("/wheel_speed", 10);
    while (pub_->get_subscription_count() == 0 && rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "Waiting for /wheel_speed subscriber");
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this]() { callback(); });

    motions_ = {Motion::Forward, Motion::Backward,  Motion::Left,
                Motion::Right,   Motion::TurnClock, Motion::TurnCounterClock,
                Motion::Stop};
    RCLCPP_INFO(get_logger(), "Initialized node");
    start_motion(0);
  }

private:
  void callback() {
    const double dt = (now() - motion_start_).seconds();

    if (dt > ramp_start_time_) {
      // Apply ramp-down to the velocities near the end of each motion interval.
      update_output_velocities();
    }
    // Publishes current wheel velocities
    publish_output_velocity();

    if (dt >= motion_duration_) {
      // Switches to the next motion
      next_motion();
    }
  }

  void publish_output_velocity() const {
    Float32MultiArray msg;
    msg.data = output_vels_;
    pub_->publish(msg);
  }

  void update_output_velocities() {
    // Linear deceleration toward zero for smooth stopping.
    const float decel = 0.15;
    for (auto &x : output_vels_) {
      if (std::abs(x) < decel)
        x = 0.0;
      else
        x -= decel * (x > 0 ? 1.0 : -1.0);
    }
  }

  void start_motion(size_t idx) {
    // Init a new motion.
    motion_count_ = idx;
    motion_start_ = now();

    target_vels_ = motion_pattern(motions_[motion_count_]);
    output_vels_ = target_vels_;

    RCLCPP_INFO(get_logger(), "%s",
                motion_name(motions_[motion_count_]).c_str());
  }

  void next_motion() {
    motion_count_++;
    if (motion_count_ >= motions_.size()) {
      timer_->cancel();
      return;
    }
    start_motion(motion_count_);
  }

  std::vector<float> motion_pattern(Motion motion) const {
    // Simplified wheel velocity patterns
    // Variable W is nominal wheel angular velocity magnitude (rad/s)
    const float W = 5.0;
    switch (motion) {
    case Motion::Forward:
      return {W, W, W, W};
    case Motion::Backward:
      return {-W, -W, -W, -W};
    case Motion::Left:
      return {-W, W, -W, W};
    case Motion::Right:
      return {W, -W, W, -W};
    case Motion::TurnClock:
      return {W, -W, -W, W};
    case Motion::TurnCounterClock:
      return {-W, W, W, -W};
    case Motion::Stop:
      return {0.0, 0.0, 0.0, 0.0};
    default: // Unknown motions
      return {0.0, 0.0, 0.0, 0.0};
    }
  }

  std::string motion_name(Motion motion) const {
    switch (motion) {
    case Motion::Forward:
      return "Move forward";
    case Motion::Backward:
      return "Move backward";
    case Motion::Left:
      return "Move left";
    case Motion::Right:
      return "Move right";
    case Motion::TurnClock:
      return "Turn clockwise";
    case Motion::TurnCounterClock:
      return "Turn counter-clockwise";
    case Motion::Stop:
      return "Stop";
    default:
      return "Unknown motion";
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_;
  rclcpp::Time motion_start_;
  std::vector<Motion> motions_;
  size_t motion_count_;
  std::vector<float> target_vels_; // Ideal velocities
  std::vector<float> output_vels_; // what gets published to wheel_vel
  const double motion_duration_;
  const double ramp_start_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}