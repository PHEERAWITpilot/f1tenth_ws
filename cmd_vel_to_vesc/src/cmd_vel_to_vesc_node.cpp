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
    
    // ✅ CORRECTED SERVO PARAMETERS
    this->declare_parameter("servo_min", 65.5);
    this->declare_parameter("servo_center", 66.0);
    this->declare_parameter("servo_max", 66.5);
    
    this->declare_parameter("wheelbase", 0.34);
    this->declare_parameter("steer_gain", 2.1);  // ← CORRECTED! ✅

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
    RCLCPP_INFO(this->get_logger(), "Servo: min=%.2f, center=%.2f, max=%.2f", 
                servo_min_, servo_center_, servo_max_);
    RCLCPP_INFO(this->get_logger(), "Steer gain: %.2f, Wheelbase: %.2f m", 
                steer_gain_, wheelbase_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    if (linear_vel < 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Negative velocity %.2f - clamping to 0", linear_vel);
      linear_vel = 0.0;
    }

    // Motor mapping
    double rpm = 0.0;
    double k_deadzone = 0.01;
    if (std::abs(linear_vel) > k_deadzone) {
      double percent = std::clamp(linear_vel, 0.0, 1.0);
      rpm = percent * (max_speed_rpm_ - min_motor_speed_) + min_motor_speed_;
      rpm = std::clamp(rpm, 0.0, max_speed_rpm_);
    }

    // Ackermann steering
    double steering_angle = 0.0;
    if (std::abs(linear_vel) > k_deadzone) {
      steering_angle = std::atan2(angular_vel * wheelbase_, std::max(std::abs(linear_vel), k_deadzone));
    }
    
    // Map steering to servo
    double servo_pos = servo_center_ + (-steer_gain_) * steering_angle;
    servo_pos = std::clamp(servo_pos, servo_min_, servo_max_);

    // Publish
    auto motor_msg = std_msgs::msg::Float64();
    motor_msg.data = rpm;
    motor_speed_pub_->publish(motor_msg);

    auto servo_msg = std_msgs::msg::Float64();
    servo_msg.data = servo_pos;
    servo_pub_->publish(servo_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "cmd_vel: lin=%.2f, ang=%.3f | steer=%.3f rad (%.1f°) | servo=%.2f | RPM=%.0f",
      linear_vel, angular_vel, steering_angle, steering_angle * 57.3, servo_pos, rpm);
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

