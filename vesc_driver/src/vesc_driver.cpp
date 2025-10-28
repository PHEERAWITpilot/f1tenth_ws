// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.hpp"
#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

//Added by BP
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
//

namespace vesc_driver
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;
using sensor_msgs::msg::Imu;

VescDriver::VescDriver(const rclcpp::NodeOptions & options)
: rclcpp::Node("vesc_driver", options),
  vesc_(
    std::string(),
    std::bind(&VescDriver::vescPacketCallback, this, _1),
    std::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(this, "duty_cycle", -1.0, 1.0),
  current_limit_(this, "current"),
  brake_limit_(this, "brake"),
  speed_limit_(this, "speed"),
  position_limit_(this, "position"),
  servo_limit_(this, "servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING),
  fw_version_major_(-1),
  fw_version_minor_(-1),
  joystick_override_active_(false),
  last_joy_time_(this->now())
{
  // get vesc serial port address
  std::string port = declare_parameter<std::string>("port", "");

  // attempt to connect to the serial port
  try {
    vesc_.connect(port);
  } catch (SerialException e) {
    RCLCPP_FATAL(get_logger(), "Failed to connect to the VESC, %s.", e.what());
    rclcpp::shutdown();
    return;
  }

  // BP Subscribe to the joy topic
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&VescDriver::joyCallback, this, std::placeholders::_1));
  //BP publish
  motor_speed_pub_ = create_publisher<Float64>("/motor_speed", rclcpp::QoS{10});
  servo_angle_pub_ = create_publisher<Float64>("/servo_angle", rclcpp::QoS{10});

  //
  // create vesc state (telemetry) publisher
  state_pub_ = create_publisher<VescStateStamped>("sensors/core", rclcpp::QoS{10});
  imu_pub_ = create_publisher<VescImuStamped>("sensors/imu", rclcpp::QoS{10});
  imu_std_pub_ = create_publisher<Imu>("sensors/imu/raw", rclcpp::QoS{10});

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = create_publisher<Float64>(
    "sensors/servo_position_command", rclcpp::QoS{10});

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = create_subscription<Float64>(
    "commands/motor/duty_cycle", rclcpp::QoS{10}, std::bind(
      &VescDriver::dutyCycleCallback, this,
      _1));
  current_sub_ = create_subscription<Float64>(
    "commands/motor/current", rclcpp::QoS{10}, std::bind(&VescDriver::currentCallback, this, _1));
  brake_sub_ = create_subscription<Float64>(
    "commands/motor/brake", rclcpp::QoS{10}, std::bind(&VescDriver::brakeCallback, this, _1));
  speed_sub_ = create_subscription<Float64>(
    "commands/motor/speed", rclcpp::QoS{10}, std::bind(&VescDriver::speedCallback, this, _1));
  position_sub_ = create_subscription<Float64>(
    "commands/motor/position", rclcpp::QoS{10}, std::bind(&VescDriver::positionCallback, this, _1));
  servo_sub_ = create_subscription<Float64>(
    "commands/servo/position", rclcpp::QoS{10}, std::bind(&VescDriver::servoCallback, this, _1));

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = create_wall_timer(20ms, std::bind(&VescDriver::timerCallback, this));
}

void VescDriver::timerCallback()
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    RCLCPP_FATAL(get_logger(), "Unexpectedly disconnected from serial port.");
    rclcpp::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      RCLCPP_INFO(
        get_logger(), "Connected to VESC with firmware version %d.%d",
        fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  } else if (driver_mode_ == MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
    // poll for vesc imu
    vesc_.requestImuData();
  } else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const std::shared_ptr<VescPacket const> & packet)
{
  if (packet->name() == "Values") {
    std::shared_ptr<VescPacketValues const> values =
      std::dynamic_pointer_cast<VescPacketValues const>(packet);

    auto state_msg = VescStateStamped();
    state_msg.header.stamp = now();

    state_msg.state.voltage_input = values->v_in();
    state_msg.state.current_motor = values->avg_motor_current();
    state_msg.state.current_input = values->avg_input_current();
    state_msg.state.avg_id = values->avg_id();
    state_msg.state.avg_iq = values->avg_iq();
    state_msg.state.duty_cycle = values->duty_cycle_now();
    state_msg.state.speed = values->rpm();

    state_msg.state.charge_drawn = values->amp_hours();
    state_msg.state.charge_regen = values->amp_hours_charged();
    state_msg.state.energy_drawn = values->watt_hours();
    state_msg.state.energy_regen = values->watt_hours_charged();
    state_msg.state.displacement = values->tachometer();
    state_msg.state.distance_traveled = values->tachometer_abs();
    state_msg.state.fault_code = values->fault_code();

    state_msg.state.pid_pos_now = values->pid_pos_now();
    state_msg.state.controller_id = values->controller_id();

    state_msg.state.ntc_temp_mos1 = values->temp_mos1();
    state_msg.state.ntc_temp_mos2 = values->temp_mos2();
    state_msg.state.ntc_temp_mos3 = values->temp_mos3();
    state_msg.state.avg_vd = values->avg_vd();
    state_msg.state.avg_vq = values->avg_vq();

    state_pub_->publish(state_msg);
  } else if (packet->name() == "FWVersion") {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
    RCLCPP_INFO(
      get_logger(),
      "-=%s=- hardware paired %d",
      fw_version->hwname().c_str(),
      fw_version->paired()
    );
  } else if (packet->name() == "ImuData") {
    std::shared_ptr<VescPacketImu const> imuData =
      std::dynamic_pointer_cast<VescPacketImu const>(packet);

    auto imu_msg = VescImuStamped();
    auto std_imu_msg = Imu();
    imu_msg.header.stamp = now();
    std_imu_msg.header.stamp = now();

    imu_msg.imu.ypr.x = imuData->roll();
    imu_msg.imu.ypr.y = imuData->pitch();
    imu_msg.imu.ypr.z = imuData->yaw();

    imu_msg.imu.linear_acceleration.x = imuData->acc_x();
    imu_msg.imu.linear_acceleration.y = imuData->acc_y();
    imu_msg.imu.linear_acceleration.z = imuData->acc_z();

    imu_msg.imu.angular_velocity.x = imuData->gyr_x();
    imu_msg.imu.angular_velocity.y = imuData->gyr_y();
    imu_msg.imu.angular_velocity.z = imuData->gyr_z();

    imu_msg.imu.compass.x = imuData->mag_x();
    imu_msg.imu.compass.y = imuData->mag_y();
    imu_msg.imu.compass.z = imuData->mag_z();

    imu_msg.imu.orientation.w = imuData->q_w();
    imu_msg.imu.orientation.x = imuData->q_x();
    imu_msg.imu.orientation.y = imuData->q_y();
    imu_msg.imu.orientation.z = imuData->q_z();

    std_imu_msg.linear_acceleration.x = imuData->acc_x();
    std_imu_msg.linear_acceleration.y = imuData->acc_y();
    std_imu_msg.linear_acceleration.z = imuData->acc_z();

    std_imu_msg.angular_velocity.x = imuData->gyr_x();
    std_imu_msg.angular_velocity.y = imuData->gyr_y();
    std_imu_msg.angular_velocity.z = imuData->gyr_z();

    std_imu_msg.orientation.w = imuData->q_w();
    std_imu_msg.orientation.x = imuData->q_x();
    std_imu_msg.orientation.y = imuData->q_y();
    std_imu_msg.orientation.z = imuData->q_z();


    imu_pub_->publish(imu_msg);
    imu_std_pub_->publish(std_imu_msg);
  }
  auto & clk = *this->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    clk,
    5000,
    "%s packet received",
    packet->name().c_str()
  );
}

void VescDriver::vescErrorCallback(const std::string & error)
{
  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
}

void VescDriver::dutyCycleCallback(const Float64::SharedPtr duty_cycle)
{
  if (joystick_override_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                         "🎮 MANUAL MODE - ignoring Nav2 duty_cycle command");
    return;
  }

  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

void VescDriver::currentCallback(const Float64::SharedPtr current)
{
  if (joystick_override_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                         "🎮 MANUAL MODE - ignoring Nav2 current command");
    return;
  }

  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

void VescDriver::brakeCallback(const Float64::SharedPtr brake)
{
  if (joystick_override_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                         "🎮 MANUAL MODE - ignoring Nav2 brake command");
    return;
  }

  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

void VescDriver::speedCallback(const Float64::SharedPtr speed)
{
  if (joystick_override_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                         "🎮 MANUAL MODE - ignoring Nav2 speed command");
    return;
  }

  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setSpeed(speed_limit_.clip(speed->data));
  }
}

void VescDriver::positionCallback(const Float64::SharedPtr position)
{
  if (joystick_override_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                         "🎮 MANUAL MODE - ignoring Nav2 position command");
    return;
  }

  if (driver_mode_ == MODE_OPERATING) {
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

void VescDriver::servoCallback(const Float64::SharedPtr servo)
{
  if (joystick_override_active_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                         "🎮 MANUAL MODE - ignoring Nav2 servo command");
    return;
  }

  if (driver_mode_ == MODE_OPERATING) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    auto servo_sensor_msg = Float64();
    servo_sensor_msg.data = servo_clipped;
    servo_sensor_pub_->publish(servo_sensor_msg);
  }
}

//Added by BP
double map(double value, double in_min, double in_max, double out_min, double out_max) {
    double mappedValue = (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    return mappedValue;
}

void VescDriver::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  last_joy_time_ = this->now();

  // 🔥 BUTTON 5 TOGGLE: Press once to switch between MANUAL and AUTO mode
  static bool last_button5_state = false;
  if (joy->buttons[5] == 1 && !last_button5_state) {  // Rising edge detection (button pressed)
    joystick_override_active_ = !joystick_override_active_;
    if (joystick_override_active_) {
      RCLCPP_WARN(get_logger(), "🎮 MANUAL MODE ACTIVATED - Press Button 5 again to return to AUTO");
    } else {
      RCLCPP_INFO(get_logger(), "🤖 AUTO MODE ACTIVATED - Nav2 control restored");
    }
  }
  last_button5_state = (joy->buttons[5] == 1);

  // 🔥 MANUAL CONTROL: Only active when in MANUAL mode
  if (joystick_override_active_) {
    double linear_vel = joy->axes[1];   // Left stick Y-axis
    double angular_vel = joy->axes[0];  // Left stick X-axis

    // Map joystick input to motor speed (±2000 RPM range)
    double motor_speed = map(linear_vel, -1.0, 1.0, -2000.0, 2000.0);
    vesc_.setSpeed(motor_speed);

    // Map joystick input to servo position (65-67 range)
    double servo_angle = map(angular_vel, -1.0, 1.0, 65.0, 67.0);
    vesc_.setServo(servo_angle);

    // Publish for monitoring
    std_msgs::msg::Float64 motor_speed_msg;
    motor_speed_msg.data = motor_speed;
    motor_speed_pub_->publish(motor_speed_msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 500,
                         "🎮 Manual: speed=%.0f RPM, servo=%.1f", 
                         motor_speed, servo_angle);

    auto servo_angle_msg = Float64();
    servo_angle_msg.data = servo_angle;
    servo_angle_pub_->publish(servo_angle_msg);
  }
}

VescDriver::CommandLimit::CommandLimit(
  rclcpp::Node * node_ptr,
  const std::string & str,
  const std::experimental::optional<double> & min_lower,
  const std::experimental::optional<double> & max_upper)
: node_ptr(node_ptr),
  logger(node_ptr->get_logger()),
  name(str)
{
  auto param_min =
    node_ptr->declare_parameter(name + "_min", rclcpp::ParameterValue(0.0));

  if (param_min.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_min.get<double>() < *min_lower) {
      lower = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_min.get<double>() > *max_upper) {
      lower = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      lower = param_min.get<double>();
    }
  } else if (min_lower) {
    lower = *min_lower;
  }

  auto param_max =
    node_ptr->declare_parameter(name + "_max", rclcpp::ParameterValue(0.0));

  if (param_max.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_max.get<double>() < *min_lower) {
      upper = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_max.get<double>() > *max_upper) {
      upper = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      upper = param_max.get<double>();
    }
  } else if (max_upper) {
    upper = *max_upper;
  }

  if (upper && lower && *lower > *upper) {
    RCLCPP_WARN_STREAM(
      logger, "Parameter " << name << "_max (" << *upper <<
        ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";

  if (lower) {
    oss << *lower << " ";
  } else {
    oss << "(none) ";
  }

  if (upper) {
    oss << *upper;
  } else {
    oss << "(none)";
  }

  RCLCPP_DEBUG_STREAM(logger, oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  auto clock = rclcpp::Clock(RCL_ROS_TIME);

  if (lower && value < lower) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) below minimum limit (%f), clipping.",
      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) above maximum limit (%f), clipping.",
      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver::VescDriver)

