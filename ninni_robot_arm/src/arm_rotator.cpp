#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

class ArmRotator : public rclcpp::Node
{
public:
  ArmRotator() : Node("arm_rotator"), t_(0.0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ArmRotator::publish_joints, this));
  }

private:
  void publish_joints()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->get_clock()->now();
    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    msg.position = {
      std::sin(t_) * 1.0,
      std::sin(t_ * 0.8) * 0.8,
      std::sin(t_ * 0.6) * 1.0,
      std::sin(t_ * 0.7) * 0.8,
      std::sin(t_ * 1.2) * 1.5
    };
    publisher_->publish(msg);
    t_ += 0.05;
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmRotator>());
  rclcpp::shutdown();
  return 0;
}
