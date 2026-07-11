#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>


class EKFCore : public rclcpp::Node {
public:
    EKFCore() : Node("ekf_core") {
        RCLCPP_INFO(this->get_logger(), "EKF Core Node Initialized");
        state_.setZero();
        state_(9) = 1.0;

        P_ = Eigen::MatrixXd::Identity(StateSize, StateSize) * 0.5;
        V_ = Eigen::MatrixXd::Zero(StateSize, StateSize);
        W_ = Eigen::MatrixXd::Zero(MeasSize, MeasSize);
        V_.diagonal() << 6.0, 6.0, 6.0, 0.5, 0.5, 0.5, 0.3, 0.3, 0.3, 0.3;
        W_.diagonal() << 1.5, 1.5, 1.5, 0.05, 0.05, 0.05, 0.05;

        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_ekf", 10);
        pose_with_cov_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_with_covariance_ekf", 10);
        gnss_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_gps_cov",
            10,
            std::bind(&EKFCore::gnss_callback, this, std::placeholders::_1)
        );
        vis_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/zed/zed_node/odom",
            10,
            std::bind(&EKFCore::vis_odom_callback, this, std::placeholders::_1)
        );
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/orient",
            10,
            std::bind(&EKFCore::imu_callback, this, std::placeholders::_1)
        );
    }

    void runEKF() {
        if (!imu_ready_) {
            return;
        }

        const rclcpp::Time now = this->get_clock()->now();
        if (!state_initialized_) {
            initialize_state();
            if (state_initialized_) {
                last_predict_time_ = now;
            }
            return;
        }

        const double dt = compute_dt(now);
        if (dt <= 0.0) {
            return;
        }

        predict(dt);
        if (gnss_ready_) {
            update_with_gnss();
            odom_ready_ = false; // Reset odometry flag to avoid using stale data
        } else if (odom_ready_) {
            update_with_odom();
        }
        
        publish_state(now);
    }
private:
    static constexpr int StateSize = 10;
    static constexpr int MeasSize = 7;
    static constexpr double Gravity = 9.83017802;
    int V_sign_;

    Eigen::VectorXd state_{StateSize};
    Eigen::MatrixXd P_{StateSize, StateSize};
    Eigen::MatrixXd V_{StateSize, StateSize};
    Eigen::MatrixXd W_{MeasSize, MeasSize};

    bool imu_ready_{false};
    bool gnss_ready_{false};
    bool odom_ready_{false};
    bool state_initialized_{false};
    rclcpp::Time last_predict_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vis_odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_;
    sensor_msgs::msg::Imu imu_;
    geometry_msgs::msg::PoseWithCovarianceStamped gnss_pose_with_covariance_;
    nav_msgs::msg::Odometry odom_;

    double compute_dt(const rclcpp::Time &now) {
        if (last_predict_time_.nanoseconds() == 0) {
            last_predict_time_ = now;
            return 0.0;
        }
        double dt = (now - last_predict_time_).seconds();
        last_predict_time_ = now;
        if (!std::isfinite(dt)) {
            return 0.0;
        }
        return std::max(0.0, std::min(dt, 0.1));
    }

    Eigen::Quaterniond get_quaternion() const {
        return Eigen::Quaterniond(state_(9), state_(6), state_(7), state_(8)).normalized();
    }

    void set_quaternion(const Eigen::Quaterniond &q) {
        state_(6) = q.x();
        state_(7) = q.y();
        state_(8) = q.z();
        state_(9) = q.w();
    }

    void initialize_state() {
        if (gnss_ready_) {
            state_.segment<3>(0) << gnss_pose_with_covariance_.pose.pose.position.x,
                gnss_pose_with_covariance_.pose.pose.position.y,
                gnss_pose_with_covariance_.pose.pose.position.z;
            const auto &o = gnss_pose_with_covariance_.pose.pose.orientation;
            Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
            if (q.norm() > 1e-6) {
                set_quaternion(q.normalized());
            }
            gnss_ready_ = false;
            state_initialized_ = true;
            return;
        }

        if (std::isfinite(imu_.orientation.w) && std::isfinite(imu_.orientation.x) &&
            std::isfinite(imu_.orientation.y) && std::isfinite(imu_.orientation.z)) {
            Eigen::Quaterniond q(imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z);
            if (q.norm() > 1e-6) {
                set_quaternion(q.normalized());
                state_initialized_ = true;
            }
        }
    }

    void predict(double dt) {
        Eigen::Vector3d pos = state_.segment<3>(0);
        Eigen::Vector3d vel = state_.segment<3>(3);
        Eigen::Quaterniond q = get_quaternion();

        const Eigen::Vector3d acc_body(imu_.linear_acceleration.x,
            imu_.linear_acceleration.y,
            imu_.linear_acceleration.z);
        const Eigen::Vector3d omega(imu_.angular_velocity.x,
            imu_.angular_velocity.y,
            imu_.angular_velocity.z);

        Eigen::Vector3d acc_world = q * acc_body;
        acc_world.z() -= Gravity;

        pos += vel * dt + 0.5 * acc_world * dt * dt;
        vel += acc_world * dt;

        const Eigen::Vector3d delta = omega * dt;
        const double angle = delta.norm();
        Eigen::Quaterniond dq;
        if (angle < 1e-6) {
            dq = Eigen::Quaterniond(1.0, 0.5 * delta.x(), 0.5 * delta.y(), 0.5 * delta.z());
        } else {
            dq = Eigen::AngleAxisd(angle, delta / angle);
        }
        q = (q * dq).normalized();

        state_.segment<3>(0) = pos;
        state_.segment<3>(3) = vel;
        set_quaternion(q);

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(StateSize, StateSize);
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        P_ = F * P_ * F.transpose() + V_ * dt;
    }

    Eigen::MatrixXd build_measurement_noise() const {
        Eigen::MatrixXd W = W_;
        const auto &cov = gnss_pose_with_covariance_.pose.covariance;
        bool has_cov = false;
        for (double value : cov) {
            if (std::isfinite(value) && value > 0.0) {
                has_cov = true;
                break;
            }
        }
        if (!has_cov) {
            return W;
        }

        W(0, 0) = std::max(1e-6, cov[0]);
        W(1, 1) = std::max(1e-6, cov[7]);
        W(2, 2) = std::max(1e-6, cov[14]);
        const double orient_var = std::max({cov[21], cov[28], cov[35], 1e-6});
        W(3, 3) = orient_var;
        W(4, 4) = orient_var;
        W(5, 5) = orient_var;
        W(6, 6) = orient_var;
        return W;
    }

    Eigen::MatrixXd build_odom_measurement_noise() const {
        Eigen::MatrixXd W = W_;
        const auto &cov = odom_.pose.covariance;
        bool has_cov = false;
        for (double value : cov) {
            if (std::isfinite(value) && value > 0.0) {
                has_cov = true;
                break;
            }
        }
        if (!has_cov) {
            return W;
        }

        W(0, 0) = std::max(1e-6, cov[0]);
        W(1, 1) = std::max(1e-6, cov[7]);
        W(2, 2) = std::max(1e-6, cov[14]);
        const double orient_var = std::max({cov[21], cov[28], cov[35], 1e-6});
        W(3, 3) = orient_var;
        W(4, 4) = orient_var;
        W(5, 5) = orient_var;
        W(6, 6) = orient_var;
        return W;
    }

    void update_with_gnss() {
        Eigen::VectorXd z(MeasSize);
        z.head<3>() << gnss_pose_with_covariance_.pose.pose.position.x,
            gnss_pose_with_covariance_.pose.pose.position.y,
            gnss_pose_with_covariance_.pose.pose.position.z;
        const auto &o = gnss_pose_with_covariance_.pose.pose.orientation;
        Eigen::Quaterniond q_meas(o.w, o.x, o.y, o.z);
        Eigen::Quaterniond q_pred = get_quaternion();
        if (q_meas.norm() <= 1e-6) {
            q_meas = q_pred;
        }
        if (q_meas.dot(q_pred) < 0.0) {
            q_meas.coeffs() *= -1.0;
        }
        z.segment<4>(3) << q_meas.x(), q_meas.y(), q_meas.z(), q_meas.w();

        Eigen::VectorXd z_hat(MeasSize);
        z_hat.head<3>() = state_.segment<3>(0);
        z_hat.segment<4>(3) << q_pred.x(), q_pred.y(), q_pred.z(), q_pred.w();

        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(MeasSize, StateSize);
        C.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        C.block<4, 4>(3, 6) = Eigen::Matrix4d::Identity();

        const Eigen::MatrixXd W = build_measurement_noise();
        const Eigen::VectorXd y = z - z_hat;
        const Eigen::MatrixXd S = C * P_ * C.transpose() + W;
        const Eigen::MatrixXd K = P_ * C.transpose() * S.inverse();

        state_ = state_ + K * y;
        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(StateSize, StateSize);
        P_ = (I - K * C) * P_;
        set_quaternion(get_quaternion());

        gnss_ready_ = false;
    }

    void update_with_odom() {
        Eigen::VectorXd z(MeasSize);
        z.head<3>() << odom_.pose.pose.position.x,
            odom_.pose.pose.position.y,
            odom_.pose.pose.position.z;
        const auto &o = odom_.pose.pose.orientation;
        Eigen::Quaterniond q_meas(o.w, o.x, o.y, o.z);
        Eigen::Quaterniond q_pred = get_quaternion();
        if (q_meas.norm() <= 1e-6) {
            q_meas = q_pred;
        }
        if (q_meas.dot(q_pred) < 0.0) {
            q_meas.coeffs() *= -1.0;
        }
        z.segment<4>(3) << q_meas.x(), q_meas.y(), q_meas.z(), q_meas.w();

        Eigen::VectorXd z_hat(MeasSize);
        z_hat.head<3>() = state_.segment<3>(0);
        z_hat.segment<4>(3) << q_pred.x(), q_pred.y(), q_pred.z(), q_pred.w();

        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(MeasSize, StateSize);
        C.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        C.block<4, 4>(3, 6) = Eigen::Matrix4d::Identity();

        const Eigen::MatrixXd W = build_odom_measurement_noise();
        const Eigen::VectorXd y = z - z_hat;
        const Eigen::MatrixXd S = C * P_ * C.transpose() + W;
        const Eigen::MatrixXd K = P_ * C.transpose() * S.inverse();

        state_ = state_ + K * y;
        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(StateSize, StateSize);
        P_ = (I - K * C) * P_;
        set_quaternion(get_quaternion());

        odom_ready_ = false;
    }

    void publish_state(const rclcpp::Time &stamp) {
        pose_.header.stamp = stamp;
        pose_.header.frame_id = gnss_pose_with_covariance_.header.frame_id.empty() ? "map"
            : gnss_pose_with_covariance_.header.frame_id;
        pose_.pose.position.x = state_(0);
        pose_.pose.position.y = state_(1);
        pose_.pose.position.z = state_(2);
        const Eigen::Quaterniond q = get_quaternion();
        pose_.pose.orientation.x = q.x();
        pose_.pose.orientation.y = q.y();
        pose_.pose.orientation.z = q.z();
        pose_.pose.orientation.w = q.w();

        pose_with_covariance_.header = pose_.header;
        pose_with_covariance_.pose.pose = pose_.pose;
        auto &cov = pose_with_covariance_.pose.covariance;
        cov.fill(0.0);
        cov[0] = P_(0, 0);
        cov[7] = P_(1, 1);
        cov[14] = P_(2, 2);
        cov[21] = P_(6, 6);
        cov[28] = P_(7, 7);
        cov[35] = P_(8, 8);

        pose_pub->publish(pose_);
        pose_with_cov_pub->publish(pose_with_covariance_);
    }

    void gnss_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        gnss_pose_with_covariance_ = *msg;
        gnss_ready_ = true;
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_ = *msg;
        imu_ready_ = true;
    }
    
    void vis_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        odom_ready_ = true;
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