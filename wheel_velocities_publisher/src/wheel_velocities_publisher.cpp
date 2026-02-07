#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher") {
    pub_ = create_publisher<Float32MultiArray>("/wheel_speed", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               [this]() { callback(); });

    motion_start_ = now();
    motions_ = {Motion::Forward, Motion::Backward,  Motion::Left,
                Motion::Right,   Motion::TurnClock, Motion::TurnCounterClock,
                Motion::Stop};
    motion_count_ = 0;
    auto first_motion = motions_[motion_count_];
    current_motion_vels_ = get_motion_velocities(first_motion);
    RCLCPP_INFO(get_logger(), "Initialized node");
    RCLCPP_INFO(get_logger(), "%s", get_motion_log(first_motion).c_str());
  }

private:
  void callback() {
    // Change motion every 3 seconds
    auto current_time = now();
    if ((current_time - motion_start_).seconds() >= 3.0) {
      motion_start_ = current_time;
      motion_count_++;
      // Clear timer when all motions are done
      if (motion_count_ >= motions_.size()) {
        timer_->cancel();
        return;
      }
      auto next_motion = motions_[motion_count_];
      current_motion_vels_ = get_motion_velocities(next_motion);
      RCLCPP_INFO(get_logger(), "%s", get_motion_log(next_motion).c_str());
    }

    Float32MultiArray msg;
    msg.data = current_motion_vels_;
    pub_->publish(msg);
  }

  std::vector<float> get_motion_velocities(Motion motion) {
    switch (motion) {
    case Motion::Forward:
      return {1.0, 1.0, 1.0, 1.0};
    case Motion::Backward:
      return {-1.0, -1.0, -1.0, -1.0};
    case Motion::Left:
      return {-1.0, 1.0, -1.0, 1.0};
    case Motion::Right:
      return {1.0, -1.0, 1.0, -1.0};
    case Motion::TurnClock:
      return {1.0, -1.0, -1.0, 1.0};
    case Motion::TurnCounterClock:
      return {-1.0, 1.0, 1.0, -1.0};
    case Motion::Stop:
      return {0.0, 0.0, 0.0, 0.0};
    default: // Unknown motions
      return {0.0, 0.0, 0.0, 0.0};
    }
  }

  std::string get_motion_log(Motion motion) {
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
  std::vector<float> current_motion_vels_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}