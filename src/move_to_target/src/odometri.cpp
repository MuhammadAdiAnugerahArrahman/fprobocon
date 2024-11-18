#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <algorithm>

class AutoMoveNode : public rclcpp::Node {
public:
    AutoMoveNode() : Node("auto_move_node"), 
        x_current_(0.0), y_current_(0.0), theta_current_(0.0),
        x_target_(0.0), y_target_(0.0), 
        target_reached_(false),
        kp_linear_(0.3), kp_angular_(0.8) {
        
        // Subscribe to odometry
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&AutoMoveNode::odomCallback, this, std::placeholders::_1));
            
        // Publisher for velocity commands
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Subscriber for target position
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_target", 10,
            std::bind(&AutoMoveNode::targetCallback, this, std::placeholders::_1));
            
        // Control loop timer (10Hz)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AutoMoveNode::controlLoop, this));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_current_ = msg->pose.pose.position.x;
        y_current_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        theta_current_ = 2.0 * atan2(qz, qw);
    }
    
    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        x_target_ = msg->pose.position.x;
        y_target_ = msg->pose.position.y;
        target_reached_ = false;
        RCLCPP_INFO(get_logger(), "New target received: x=%.2f, y=%.2f", x_target_, y_target_);
    }
    
    void controlLoop() {
        if (target_reached_) return;
        
        // Calculate errors
        double dx = x_target_ - x_current_;
        double dy = y_target_ - y_current_;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate target heading
        double target_heading = std::atan2(dy, dx);
        double heading_error = target_heading - theta_current_;
        
        // Normalize heading error
        while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
        
        // Generate control commands
        geometry_msgs::msg::Twist cmd_vel;
        
        if (distance > 0.1) {  // If not at target
            cmd_vel.linear.x = kp_linear_ * distance;
            cmd_vel.angular.z = kp_angular_ * heading_error;
            
            // Limit velocities
            cmd_vel.linear.x = std::min(cmd_vel.linear.x, 0.5);
            cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -1.0, 1.0);
        } else {
            target_reached_ = true;
            RCLCPP_INFO(get_logger(), "Target reached!");
        }
        
        cmd_vel_pub_->publish(cmd_vel);
    }

    // Current state
    double x_current_, y_current_, theta_current_;
    double x_target_, y_target_;
    bool target_reached_;
    
    // PID gains
    const double kp_linear_;
    const double kp_angular_;
    
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoMoveNode>());
    rclcpp::shutdown();
    return 0;
}