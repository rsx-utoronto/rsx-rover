#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std;

class EKFCore : public rclcpp::Node {
public:
    EKFCore() : Node("ekf_core") {
        RCLCPP_INFO(this->get_logger(), "EKF Core Node Initialized");
        // Initialize EKF parameters and state here
    }

    void runEKF() {
        // EKF algorithm implementation goes here
        RCLCPP_INFO(this->get_logger(), "Running EKF step...");
    }
private:
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist twist;
    void callback() {
        // Placeholder for callback functionality
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto ekf_node = std::make_shared<EKFCore>();

    // Eigen::MatrixXd m(2, 2);
    // m(0, 0) = 3;
    // m(1, 0) = 2.5;
    // m(0, 1) = -1;
    // m(1, 1) = m(1, 0) + m(0, 1);
    // std::cout << m << std::endl;

    rclcpp::Rate loop_rate(10); // 10 Hz
    while (rclcpp::ok()) {
        ekf_node->runEKF();
        rclcpp::spin_some(ekf_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}