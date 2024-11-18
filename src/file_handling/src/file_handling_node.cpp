#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <iostream>
#include <fstream> // Untuk file handling
#include <thread>  // Untuk sleep
#include <chrono>

using namespace std;

class FileHandlingNode : public rclcpp::Node
{
public:
    FileHandlingNode() : Node("file_handling_node"), x_current_(0.0), y_current_(0.0)
    {
        // Membuat publisher untuk perintah ke robot
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer untuk mengecek pergerakan robot setiap 100ms
        timer_ = this->create_wall_timer(
            chrono::milliseconds(100),
            std::bind(&FileHandlingNode::move_to_target, this)
        );
    }

    // Fungsi untuk membaca file dan memuat target-target
    bool load_targets_from_file(const string &filename)
    {
        ifstream file("/home/user/fprobocon/src/file_handling/src/input.txt");
        if (!file.is_open()) {
            cerr << "Gagal membuka file: " << filename << endl;
            return false;
        }

        double x, y;
        while (file >> x >> y) {
            target_list_.push_back(make_pair(x, y));
        }

        file.close();
        return true;
    }

    // Fungsi untuk menjalankan logika bergerak menuju target
    void move_to_target()
    {
        if (current_target_index_ < target_list_.size()) {
            double x_target = target_list_[current_target_index_].first;
            double y_target = target_list_[current_target_index_].second;

            // Hitung error posisi
            double error_x = x_target - x_current_;
            double error_y = y_target - y_current_;

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
            cout << "Robot bergerak ke (" << x_target << ", " << y_target << ") dengan v = " 
                 << v << " dan omega = " << joder << endl;

            // Cek apakah robot sudah cukup dekat dengan target
            if (sqrt(error_x * error_x + error_y * error_y) < 0.1) {  // Threshold jarak (misalnya 0.1 meter)
                cout << "Robot sudah mencapai target!" << endl;
                // Stop the robot
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;
                cmd_vel_publisher_->publish(stop_cmd);
                //reset posisi robot
                x_current_ = 0.0;
                y_current_ = 0.0;
                // Pindah ke target berikutnya
                current_target_index_++;
            }else{
              x_current_ += v * cos(theta) * 0.1;  // Update posisi x
              y_current_ += v * sin(theta) * 0.1;  // Update posisi y
            }
        } else {
            // Semua target sudah dicapai, berhenti
            cout << "Semua target telah tercapai." << endl;
            rclcpp::shutdown();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variabel untuk posisi robot saat ini dan target
    double x_current_, y_current_;
    vector<pair<double, double>> target_list_; // Daftar target (x, y)
    size_t current_target_index_ = 0; // Index target saat ini
};

int main(int argc, char **argv)
{
    // Inisialisasi ROS2
    rclcpp::init(argc, argv);

    // Membuat objek node
    auto node = std::make_shared<FileHandlingNode>();
    // Memuat target dari file input.txt
    if (!node->load_targets_from_file("/home/user/fprobocon/src/file_handling/src/input.txt")) {
        cerr << "Gagal memuat target dari file!" << endl;
        return 1;
    }

    // Menjalankan node
    rclcpp::spin(node);

    // Menutup ROS2
    rclcpp::shutdown();
    return 0;
}
