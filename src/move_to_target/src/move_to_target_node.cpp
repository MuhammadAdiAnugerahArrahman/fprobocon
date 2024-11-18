#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <iostream>
#include <fstream>  // Untuk membaca file
#include <thread>   // Untuk sleep
#include <chrono>

using namespace std;

class MoveToTargetNode : public rclcpp::Node
{
public:
    MoveToTargetNode() : Node("move_to_target_node"), x_current_(0.0), y_current_(0.0), x_target_(0.0), y_target_(0.0), target_reached_(false)
    {
        // Membuat publisher untuk perintah ke robot
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer akan dibuat saat diperlukan
    }

    // Fungsi untuk menjalankan logika bergerak menuju target
    void move_to_target()
    {
        // Hitung error posisi
        double error_x = x_target_ - x_current_;
        double error_y = y_target_ - y_current_;

        // Hitung arah robot ke target (dengan atan2 untuk mencari sudut)
        double theta = atan2(error_y, error_x);

        // Hitung kecepatan linier dan angular
        double v = 0.5 * sqrt(error_x * error_x + error_y * error_y);  // Kecepatan linier
        double joder = 0.5 * theta;  // Kecepatan angular

        // Membuat pesan untuk mengontrol robot
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = v;
        cmd_vel.angular.z = joder;

        // Mempublikasikan perintah ke robot
        cmd_vel_publisher_->publish(cmd_vel);

        // Feedback ke terminal
        cout << "Robot bergerak ke (" << x_target_ << ", " << y_target_ << ") dengan v = "
             << v << " dan omega = " << joder << endl;

        // Cek apakah robot sudah cukup dekat dengan target
        if (sqrt(error_x * error_x + error_y * error_y) < 0.1) {  // Threshold jarak (misalnya 0.1 meter)
            cout << "Robot sudah mencapai target!" << endl;
            // Stop the robot
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_publisher_->publish(stop_cmd);

            // Hentikan timer setelah robot mencapai target
            timer_->cancel();

            // Set flag bahwa target telah tercapai
            target_reached_ = true;
        }

        // Perbarui posisi robot berdasarkan kecepatan
        x_current_ += v * cos(theta) * 0.1;  // Update posisi x
        y_current_ += v * sin(theta) * 0.1;  // Update posisi y
    }

    // Fungsi untuk set target x dan y
    void set_target(double x, double y)
    {
        x_target_ = x;
        y_target_ = y;
        x_current_ = 0.0;  // Reset posisi robot (jika diperlukan)
        y_current_ = 0.0;
        target_reached_ = false;  // Reset flag target tercapai

        // Membuat timer baru untuk pergerakan ke target baru
        timer_ = this->create_wall_timer(
            chrono::milliseconds(100),
            std::bind(&MoveToTargetNode::move_to_target, this)
        );
    }

    // Fungsi untuk mengecek apakah target telah tercapai
    bool is_target_reached() const
    {
        return target_reached_;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    double x_current_;
    double y_current_;
    double x_target_;
    double y_target_;
    bool target_reached_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveToTargetNode>();

    // Membaca koordinat target dari file
    ifstream file("/home/user/fprobocon/src/file_handling/src/input.txt");  // Pastikan path sesuai dengan lokasi file Anda
    if (!file.is_open()) {
        cout << "Gagal membuka file: input.txt" << endl;
        return 1;
    }

    double x_target, y_target;
    while (file >> x_target >> y_target) {
        // Set target pada node untuk setiap koordinat yang dibaca dari file
        node->set_target(x_target, y_target);

        // Spin node sampai target tercapai
        while (rclcpp::ok() && !node->is_target_reached()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Menghindari penggunaan CPU berlebih
        }

        // Beri jeda 5 detik sebelum bergerak ke target berikutnya
        cout << "Menunggu selama 5 detik sebelum bergerak ke target berikutnya..." << endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    file.close();  // Menutup file setelah selesai

    // Menutup ROS2
    rclcpp::shutdown();
    return 0;
}
