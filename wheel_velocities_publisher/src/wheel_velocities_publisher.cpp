#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
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
  WheelVelocitiesPublisher()
      : Node("wheel_velocities_publisher"), motion_count_(0) {
    pub_ = create_publisher<Float32MultiArray>("/wheel_speed", 10);
    timer_ =
        create_wall_timer(std::chrono::seconds(3), [this]() { callback(); });

    motions_ = {Motion::Forward, Motion::Backward,  Motion::Left,
                Motion::Right,   Motion::TurnClock, Motion::TurnCounterClock,
                Motion::Stop};
    RCLCPP_INFO(get_logger(), "Initialized node");
  }

private:
  void callback() {
    if (motion_count_ == motions_.size()) {
      timer_->cancel();
      return;
    }
    auto [motion, log_s] = get_velocities_by_motion(motions_[motion_count_++]);
    RCLCPP_INFO(get_logger(), "%s", log_s.c_str());
    auto msg = Float32MultiArray();
    msg.data = motion;
    pub_->publish(msg);
  }

  std::pair<std::vector<float>, std::string>
  get_velocities_by_motion(Motion motion) {
    switch (motion) {
    case Motion::Forward:
      return {{1.0, 1.0, 1.0, 1.0}, "Move forward"};
    case Motion::Backward:
      return {{-1.0, -1.0, -1.0, -1.0}, "Move backward"};
    case Motion::Left:
      return {{-1.0, 1.0, -1.0, 1.0}, "Move left"};
    case Motion::Right:
      return {{1.0, -1.0, 1.0, -1.0}, "Move right"};
    case Motion::TurnClock:
      return {{1.0, -1.0, -1.0, 1.0}, "Turn clockwise"};
    case Motion::TurnCounterClock:
      return {{-1.0, 1.0, 1.0, -1.0}, "Turn counter-clockwise"};
    case Motion::Stop:
      return {{0.0, 0.0, 0.0, 0.0}, "Stop"};
    default:
      return {{0.0, 0.0, 0.0, 0.0}, "Unknown motion"};
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_;
  std::vector<Motion> motions_;
  size_t motion_count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
  rclcpp::shutdown();
  return 0;
}