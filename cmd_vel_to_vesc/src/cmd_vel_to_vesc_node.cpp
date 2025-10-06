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
    this->declare_parameter("min_motor_speed", 2500.0);
    this->declare_parameter("max_speed_rpm", 6000.0);
    
    // *** UPDATED: Changed servo range from 0.0-1.0 to 65.0-67.0 to match hardware ***
    this->declare_parameter("servo_min", 65.0);      // Full left (hardware min)
    this->declare_parameter("servo_center", 66.0);   // Center (midpoint)
    this->declare_parameter("servo_max", 67.0);      // Full right (hardware max)
    
    this->declare_parameter("wheelbase", 0.3);       // In meters, measure your robot!
    this->declare_parameter("steer_gain", 3.37);      // Gain mapping radian to servo range

    min_motor_speed_ = this->get_parameter("min_motor_speed").as_double();
    max_speed_rpm_ = this->get_parameter("max_speed_rpm").as_double();
    servo_min_ = this->get_parameter("servo_min").as_double();
    servo_center_ = this->get_parameter("servo_center").as_double();
    servo_max_ = this->get_parameter("servo_max").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    steer_gain_ = this->get_parameter("steer_gain").as_double();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelToVesc::cmdVelCallback, this, std::placeholders::_1));
    motor_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 10);
    servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 10);

    RCLCPP_INFO(this->get_logger(), "--- cmd_vel_to_vesc node ready ---");
    RCLCPP_INFO(this->get_logger(), "Servo range configured: min=%.1f, center=%.1f, max=%.1f", 
                servo_min_, servo_center_, servo_max_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    // Motor mapping - simple proportional, with deadzone
    double rpm = 0.0;
    double k_deadzone = 0.01;
    if (std::abs(linear_vel) > k_deadzone) {
      double percent = std::clamp(linear_vel, -1.0, 1.0); // If using normalized [-1,1] in nav stack
      rpm = percent * (max_speed_rpm_ - min_motor_speed_) + (percent > 0 ? min_motor_speed_ : -min_motor_speed_);
      rpm = std::clamp(rpm, -max_speed_rpm_, max_speed_rpm_);
    }

    // Ackermann steering: Convert angular_vel and linear_vel to steering angle (radian)
    double steering_angle = 0.0;
    if (std::abs(linear_vel) > k_deadzone) {
      steering_angle = std::atan2(angular_vel * wheelbase_, std::max(std::abs(linear_vel), k_deadzone));
    }
    
    // Map steering angle (rad) to servo value in 65-67 range
    // steer_gain should be positive or negative to match your robot direction!
    double servo_pos = servo_center_ + (-steer_gain_) * steering_angle;

    // Clamp to hardware limits (65-67)
    servo_pos = std::clamp(servo_pos, servo_min_, servo_max_);

    // Publish
    auto motor_msg = std_msgs::msg::Float64();
    motor_msg.data = rpm;
    motor_speed_pub_->publish(motor_msg);

    auto servo_msg = std_msgs::msg::Float64();
    servo_msg.data = servo_pos;
    servo_pub_->publish(servo_msg);

    RCLCPP_INFO(this->get_logger(),
      "cmd_vel: %.2f m/s, %.2f rad/s --> RPM: %.0f, servo: %.2f, steering_angle(rad): %.3f",
      linear_vel, angular_vel, rpm, servo_pos, steering_angle);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_speed_pub_, servo_pub_;
  double min_motor_speed_, max_speed_rpm_;
  double servo_min_, servo_center_, servo_max_;
  double wheelbase_, steer_gain_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToVesc>());
  rclcpp::shutdown();
  return 0;
}

