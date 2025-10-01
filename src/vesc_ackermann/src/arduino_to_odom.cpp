#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace vesc_ackermann
{

class ArduinoToOdom : public rclcpp::Node
{
public:
  ArduinoToOdom(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
    : Node("vesc_to_odom_node", options),  // Renamed to reflect VESC usage
      x_(0.0), y_(0.0), yaw_(0.0), vel_(0.0),
      got_initial_yaw_(false), last_yaw_(0.0), got_initial_vel_(false)
  {
    RCLCPP_INFO(this->get_logger(), "✅ VESC to Odom node starting...");

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10); 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this); 

    // Subscribers - Using correct topics with signed velocity
    vel_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/vel", 10,
        std::bind(&ArduinoToOdom::velCallback, this, std::placeholders::_1));

    imu_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/imu/yaw", 10,
        std::bind(&ArduinoToOdom::imuYawCallback, this, std::placeholders::_1));

    last_time_ = this->now();

    // Timer for periodic odometry update (20Hz)
    timer_ = this->create_wall_timer(
        50ms, std::bind(&ArduinoToOdom::updateOdom, this));
        
    RCLCPP_INFO(this->get_logger(), "✅ VESC to Odom node started successfully");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /imu/yaw (IMU), /vel (VESC signed velocity)");
    RCLCPP_INFO(this->get_logger(), "Publishing: /odom and TF odom->base_link");
  }

private:
  // Member variables
  double last_yaw_;
  double x_, y_, yaw_;
  double vel_;  // Now stores signed velocity

  // Initialization flags
  double initial_yaw_;
  bool got_initial_yaw_;
  bool got_initial_vel_;

  rclcpp::Time last_time_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr imu_yaw_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void velCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // ✅ Store signed velocity directly (+ = forward, - = backward)
    vel_ = msg->data;
    
    if (!got_initial_vel_) {
      got_initial_vel_ = true;
      RCLCPP_INFO(this->get_logger(), "✅ First velocity reading: %.3f m/s", vel_);
    }
    
    // Enhanced debug logging with direction
    std::string direction = vel_ > 0.01 ? "FWD" : vel_ < -0.01 ? "REV" : "STOP";
    RCLCPP_DEBUG(this->get_logger(), "Signed velocity: %.3f m/s (%s)", vel_, direction.c_str());
  }

  void imuYawCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    double raw_yaw = msg->data; // Yaw in degrees from Arduino

    // Convert degrees to radians
    double yaw_rad = raw_yaw * M_PI / 180.0;

    if (!got_initial_yaw_) {
      initial_yaw_ = yaw_rad;
      got_initial_yaw_ = true;
      RCLCPP_INFO(this->get_logger(), "✅ Initial yaw set: %.1f deg", raw_yaw);
    }

    // Calculate relative yaw and normalize
    yaw_ = normalizeAngle(yaw_rad - initial_yaw_);
    RCLCPP_DEBUG(this->get_logger(), "IMU yaw: raw=%.1f°, relative=%.1f°", 
                 raw_yaw, yaw_ * 180.0 / M_PI);
  }

  void updateOdom() {
    // Wait for both IMU and velocity data
    if (!got_initial_yaw_) {
      static int yaw_warning_counter = 0;
      if (++yaw_warning_counter % 100 == 0) { // Every 5 seconds at 20Hz
        RCLCPP_WARN(this->get_logger(), "Waiting for initial IMU yaw data from /imu/yaw...");
      }
      return;
    }

    if (!got_initial_vel_) {
      static int vel_warning_counter = 0;
      if (++vel_warning_counter % 100 == 0) { // Every 5 seconds at 20Hz
        RCLCPP_WARN(this->get_logger(), "Waiting for velocity data from /vel...");
      }
      return;
    }

    auto now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;
    last_time_ = now;

    // Calculate angular velocity from yaw change
    double angular_vel = normalizeAngle(yaw_ - last_yaw_) / dt;
    last_yaw_ = yaw_;

    // ✅ Use signed velocity directly from VESC speed calculator
    double signed_vel = vel_;  // Already has correct sign (+ = forward, - = backward)

    // Update position using odometry integration
    // The velocity is in the robot's local frame (forward/backward)
    // Transform to global frame using current yaw
    double dx = signed_vel * dt * std::cos(yaw_);
    double dy = signed_vel * dt * std::sin(yaw_);
    x_ += dx;
    y_ += dy;

    // Prepare odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation from yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Velocity (in base_link frame)
    odom_msg.twist.twist.linear.x = signed_vel;  // Forward/backward velocity
    odom_msg.twist.twist.linear.y = 0.0;         // No sideways motion
    odom_msg.twist.twist.angular.z = angular_vel; // Yaw rate

    // Initialize covariance matrices to zero
    for (int i = 0; i < 36; i++) {
      odom_msg.pose.covariance[i] = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }
    
    // Set diagonal elements for uncertainty estimates
    odom_msg.pose.covariance[0] = 0.01;   // x position variance
    odom_msg.pose.covariance[7] = 0.01;   // y position variance  
    odom_msg.pose.covariance[35] = 0.01;  // yaw variance
    odom_msg.twist.covariance[0] = 0.01;  // x velocity variance
    odom_msg.twist.covariance[35] = 0.01; // yaw velocity variance

    // Publish odometry
    odom_pub_->publish(odom_msg);

    // Broadcast transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform);
    
    // Enhanced debug output every 2 seconds
    static int debug_counter = 0;
    if (++debug_counter >= 40) { // 40 * 50ms = 2 seconds
      std::string direction = signed_vel > 0.01 ? "FWD" : signed_vel < -0.01 ? "REV" : "STOP";
      RCLCPP_INFO(this->get_logger(), 
                  "Odom: pos(%.2f, %.2f)m, yaw=%.1f°, vel=%.3fm/s (%s), ω=%.2frad/s", 
                  x_, y_, yaw_ * 180.0 / M_PI, signed_vel, direction.c_str(), angular_vel);
      debug_counter = 0;
    }

    // Additional debug for movement detection with direction
    if (std::abs(dx) > 0.001 || std::abs(dy) > 0.001) {
      std::string move_dir = signed_vel > 0 ? "forward" : "backward";
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Moving %s: dx=%.4f, dy=%.4f, vel=%.3f, yaw=%.1f°", 
                          move_dir.c_str(), dx, dy, signed_vel, yaw_ * 180.0 / M_PI);
    }
  }

  // Helper function to normalize angle between -pi and +pi
  double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
};

} // namespace vesc_ackermann

// Component registration for ROS2
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::ArduinoToOdom)

// Main function for standalone execution
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<vesc_ackermann::ArduinoToOdom>();
  
  try {
    RCLCPP_INFO(node->get_logger(), "Starting VESC to Odom node...");
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_to_odom"), "Exception: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}

