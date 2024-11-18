#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>
class PointReaderNode : public rclcpp::Node {
public:
    PointReaderNode() : Node("point_reader"), currejnt_point_(0) {
        // Publisher for target points
        target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/move_target", 10);
        
        // Subscribe to odometry to check if target is reached
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PointReaderNode::odom_callback, this, std::placeholders::_1));

        // Get package share directory
        const std::string pkg_share_dir = get_package_share_directory();
        const std::string points_file = pkg_share_dir + "/config/points.txt";
        
        RCLCPP_INFO(get_logger(), "Reading points from: %s", points_file.c_str());
        readPointsFromFile(points_file);

        // Timer to check progress and publish next point
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PointReaderNode::timer_callback, this));
    }

private:
    std::string get_package_share_directory() {
        const char* share_dir = std::getenv("COLCON_PREFIX_PATH");
        if (share_dir) {
            return std::string(share_dir) + "/file_handling/share/file_handling";
        }
        return "/opt/ros/humble/share/file_handling";
    }

    void readPointsFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }

        double x, y;
        while (file >> x >> y) {
            Point point;
            point.x = x;
            point.y = y;
            points_.push_back(point);
        }
        
        RCLCPP_INFO(get_logger(), "Loaded %ld points from file", points_.size());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (current_point_ >= points_.size()) return;

        // Check if we're close enough to current target
        double dx = msg->pose.pose.position.x - points_[current_point_].x;
        double dy = msg->pose.pose.position.y - points_[current_point_].y;
        double distance = std::sqrt(dx*dx + dy*dy);

        if (distance < 0.1) { // Within 10cm
            current_point_++;
            if (current_point_ < points_.size()) {
                publishNextPoint();
            } else {
                RCLCPP_INFO(get_logger(), "All points reached!");
            }
        }
    }

    void timer_callback() {
        if (current_point_ == 0) {
            publishNextPoint();
        }
    }

    void publishNextPoint() {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "map";
        target.header.stamp = now();
        target.pose.position.x = points_[current_point_].x;
        target.pose.position.y = points_[current_point_].y;
        target.pose.position.z = 0.0;
        target.pose.orientation.w = 1.0;

        target_pub_->publish(target);
        RCLCPP_INFO(get_logger(), "Publishing point %ld: (%.2f, %.2f)", 
                    current_point_ + 1, points_[current_point_].x, points_[current_point_].y);
    }

    struct Point {
        double x;
        double y;
    };

    std::vector<Point> points_;
    size_t current_point_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointReaderNode>());
    rclcpp::shutdown();
    return 0;
}