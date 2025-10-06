#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <algorithm>

class CmdVelToVesc : public rclcpp::Node
{
public:
  CmdVelToVesc() : Node("cmd_vel_to_vesc")
  {
    // Declare parameters with VESC-compatible 0.0-1.0 range
    this->declare_parameter("min_motor_speed", 2500.0);
    this->declare_parameter("max_speed_rpm", 6000.0);
    this->declare_parameter("servo_min", 0.0);      // Full left
    this->declare_parameter("servo_center", 0.5);   // Center
    this->declare_parameter("servo_max", 1.0);      // Full right

    // Get parameters
    min_motor_speed_ = this->get_parameter("min_motor_speed").as_double();
    max_speed_rpm_ = this->get_parameter("max_speed_rpm").as_double();
    servo_min_ = this->get_parameter("servo_min").as_double();
    servo_center_ = this->get_parameter("servo_center").as_double();
    servo_max_ = this->get_parameter("servo_max").as_double();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelToVesc::cmdVelCallback, this, std::placeholders::_1));
    motor_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 10);
    servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 10);

    RCLCPP_INFO(this->get_logger(), "--- cmd_vel_to_vesc node ready ---\nMin RPM: %.1f, Max RPM: %.1f, Servo: %.2f/%.2f/%.2f",
      min_motor_speed_, max_speed_rpm_, servo_min_, servo_center_, servo_max_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    // Motor control
    double rpm = 0.0;
    if (std::abs(linear_vel) > 1e-3) {
      rpm = linear_vel > 0
        ? std::max(min_motor_speed_, std::abs(linear_vel) * max_speed_rpm_)
        : -std::max(min_motor_speed_, std::abs(linear_vel) * max_speed_rpm_);
      rpm = std::clamp(rpm, -max_speed_rpm_, max_speed_rpm_);
    }

    // Servo mapping for VESC (0.0 to 1.0 range)
    double servo_pos;
    if (std::abs(angular_vel) < 1e-3) {
      servo_pos = servo_center_;
    } else {
      servo_pos = servo_center_ + 
                  (servo_max_ - servo_center_) * std::max(0.0, -angular_vel) +
                  (servo_min_ - servo_center_) * std::max(0.0, angular_vel);
    }
    servo_pos = std::clamp(servo_pos, servo_min_, servo_max_);

    // Publish
    auto motor_msg = std_msgs::msg::Float64(); 
    motor_msg.data = rpm;
    motor_speed_pub_->publish(motor_msg);
    
    auto servo_msg = std_msgs::msg::Float64(); 
    servo_msg.data = servo_pos;
    servo_pub_->publish(servo_msg);

    RCLCPP_INFO(this->get_logger(), "cmd_vel: %.2f m/s, %.2f rad/s --> RPM: %.0f, Servo: %.3f",
                linear_vel, angular_vel, rpm, servo_pos);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;

  double min_motor_speed_, max_speed_rpm_;
  double servo_min_, servo_center_, servo_max_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToVesc>());
  rclcpp::shutdown();
  return 0;
}

