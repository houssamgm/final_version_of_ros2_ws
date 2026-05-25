#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <utility>
#include <functional>
#include <chrono>

using std::placeholders::_1;

class ACCFollow : public rclcpp::Node
{
public:
    ACCFollow()
    : Node("acc_follow"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // ---------- ACC parameters ----------
        // Constant Time Headway Spacing Policy
        headway_time_ = 0.8;   // seconds
        min_distance_ = 0.5;   // absolute minimum safe gap

        // Controller Gains
        Kp_acc_ = 3;         // Gap error proportional gain
        Kv_acc_ = 1;         // Relative velocity derivative gain
        Kp_angle_ = 3.0;       // Heading gain

        v_max_ = 0.6;
        w_max_ = 3.0;

        // ACC Memory & Filtering
        prev_distance_ = min_distance_;
        filtered_rel_vel_ = 0.0;
        vel_filter_alpha_ = 0.25; // Low-pass filter coefficient for noisy derivative
        prev_time_ = this->get_clock()->now();

        // ---------- DWA safety parameters ----------
        max_accel_ = 0.6;
        max_delta_w_ = 1.5;
        predict_time_ = 0.8;
        dt_sim_ = 0.1;
        v_reso_ = 0.03;
        w_reso_ = 0.10;
        robot_radius_ = 0.22;

        prev_v_ = 0.0;
        prev_w_ = 0.0;
        smooth_gain_ = 0.2;

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ACCFollow::scan_callback, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ACCFollow::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "ACC follower with heading error metric started");
    }

private:
    // ---------- Parameters ----------
    // ACC gains & spacing
    double headway_time_;
    double min_distance_;
    double Kp_acc_;
    double Kv_acc_;
    double Kp_angle_;

    double v_max_;
    double w_max_;

    double max_accel_;
    double max_delta_w_;
    double predict_time_;
    double dt_sim_;
    double v_reso_;
    double w_reso_;
    double robot_radius_;

    // ===== Metrics and Oscillation Tracking Variables =====
    bool reached_setpoint_ = false;
    double zero_band_ = 0.02;
    int oscillation_count_ = 0;
    double max_oscillation_ = 0.0;
    int prev_sign_ = 0;

    // ACC Memory
    double prev_distance_;
    double filtered_rel_vel_;
    double vel_filter_alpha_;
    rclcpp::Time prev_time_;

    // smoothing
    double prev_v_ = 0.0;
    double prev_w_ = 0.0;
    double smooth_gain_ = 0.2;

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // --------------------------------------------------
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = msg;
    }

    // --------------------------------------------------
    std::vector<std::pair<double, double>> predict_trajectory(double v, double w)
    {
        std::vector<std::pair<double, double>> traj;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;

        double t = 0.0;
        while (t <= predict_time_)
        {
            x += v * std::cos(yaw) * dt_sim_;
            y += v * std::sin(yaw) * dt_sim_;
            yaw += w * dt_sim_;
            traj.emplace_back(x, y);
            t += dt_sim_;
        }

        return traj;
    }

    // --------------------------------------------------
    double calc_obstacle_cost(const std::vector<std::pair<double, double>> & traj)
    {
        if (!scan_)
        {
            return 0.0;
        }

        double min_dist = std::numeric_limits<double>::infinity();

        for (const auto & point : traj)
        {
            for (size_t i = 0; i < scan_->ranges.size(); ++i)
            {
                double r = scan_->ranges[i];
                if (std::isinf(r) || std::isnan(r)) continue;

                double angle = scan_->angle_min + i * scan_->angle_increment;

                double obs_x = r * std::cos(angle);
                double obs_y = r * std::sin(angle);

                double dist = std::hypot(point.first - obs_x, point.second - obs_y);

                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        if (std::isinf(min_dist)) return 0.0;
        if (min_dist < robot_radius_) return std::numeric_limits<double>::infinity();

        return 1.0 / (min_dist + 0.01);
    }

    // --------------------------------------------------
    std::pair<double, double> dwa_safety_filter(double v_cmd, double w_cmd)
    {
        if (!scan_) return {v_cmd, w_cmd};

        double min_v = std::max(-v_max_, v_cmd - max_accel_ * dt_sim_);
        double max_v = std::min(v_max_,  v_cmd + max_accel_ * dt_sim_);

        double angle_expand = (std::abs(v_cmd) > 0.05) ? 0.7 : 0.4;

        double min_w = std::max(-w_max_, w_cmd - max_delta_w_ * dt_sim_ - angle_expand);
        double max_w = std::min(w_max_,  w_cmd + max_delta_w_ * dt_sim_ + angle_expand);

        double best_v = 0.0;
        double best_w = 0.0;
        double min_cost = std::numeric_limits<double>::infinity();
        bool found = false;

        int n_v = static_cast<int>(std::floor((max_v - min_v) / v_reso_));
        int n_w = static_cast<int>(std::floor((max_w - min_w) / w_reso_));

        for (int i = 0; i <= n_v + 1; ++i)
        {
            double v = min_v + i * v_reso_;
            if (v > max_v + v_reso_ * 0.5) continue;

            for (int j = 0; j <= n_w + 1; ++j)
            {
                double w = min_w + j * w_reso_;
                if (w > max_w + w_reso_ * 0.5) continue;

                auto traj = predict_trajectory(v, w);
                double obstacle_cost = calc_obstacle_cost(traj);

                if (std::isinf(obstacle_cost)) continue;

                double tracking_cost = std::abs(v - v_cmd) + std::abs(w - w_cmd);
                double final_cost = 3.5 * obstacle_cost + 4.0 * tracking_cost;

                if (final_cost < min_cost)
                {
                    min_cost = final_cost;
                    best_v = v;
                    best_w = w;
                    found = true;
                }
            }
        }

        if (!found)
            return {0.0, 0.8};

        return {best_v, best_w};
    }

    int sign_of(double x)
    {
        if (x > zero_band_) return 1;
        if (x < -zero_band_) return -1;
        return 0;
    }

    // --------------------------------------------------
    void control_loop()
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        rclcpp::Time now = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped trans;

        try
        {
            trans = tf_buffer_.lookupTransform(
                "base_link",
                "leader/base_link",
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &)
        {
            return;
        }

        double dx = trans.transform.translation.x;
        double dy = trans.transform.translation.y;

        double distance = std::sqrt(dx * dx + dy * dy);

        // =========================================================
        //                🚗 KINEMATIC ACC CONTROLLER
        // =========================================================
        
        // Calculate dt for derivative
        double dt = (now - prev_time_).nanoseconds() / 1e9;
        if (dt <= 0.0) return;

        // Estimate relative velocity & apply low-pass filter to smooth TF noise
        double raw_rel_vel = (distance - prev_distance_) / dt;
        filtered_rel_vel_ = vel_filter_alpha_ * raw_rel_vel + (1.0 - vel_filter_alpha_) * filtered_rel_vel_;

        // 1. Determine Desired Gap (Constant Time Headway policy)
        double desired_gap = min_distance_ + headway_time_ * std::max(0.0, prev_v_);
        
        // 2. Calculate Gap Error
        double gap_error = distance - desired_gap;

        // 3. ACC Control Law (PD on gap distance)
        double v_acc = Kp_acc_ * gap_error + Kv_acc_ * filtered_rel_vel_;

        double v_raw = std::clamp(v_acc, -v_max_, v_max_);

        // Angular controller
        double angle_error = std::atan2(dy, dx);
        double heading_err_abs = std::abs(angle_error);
        double w_raw = Kp_angle_ * angle_error;
        w_raw = std::clamp(w_raw, -w_max_, w_max_);

        // Update memory for next loop
        prev_distance_ = distance;
        prev_time_ = now;

        // =========================================================
        // DWA layer
        // =========================================================
        auto filtered = dwa_safety_filter(v_raw, w_raw);

        double v = (1.0 - smooth_gain_) * filtered.first + smooth_gain_ * prev_v_;
        double w = (1.0 - smooth_gain_) * filtered.second + smooth_gain_ * prev_w_;

        prev_v_ = v;
        prev_w_ = w;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;

        cmd_pub_->publish(cmd);

        auto loop_end = std::chrono::high_resolution_clock::now();
        double loop_time_ms =
            std::chrono::duration<double, std::milli>(loop_end - loop_start).count();

        double err_abs = std::abs(gap_error);

        // --- Start detection after reaching setpoint zone ---
        if (!reached_setpoint_ && std::abs(gap_error) < zero_band_)
        {
            reached_setpoint_ = true;
            prev_sign_ = 0;
        }

        // --- Oscillation Tracking Logic ---
        if (reached_setpoint_)
        {
            int current_sign = sign_of(gap_error);

            if (current_sign != 0)
            {
                if (std::abs(gap_error) > max_oscillation_)
                {
                    max_oscillation_ = std::abs(gap_error);
                }
            }

            if (current_sign != 0 && prev_sign_ != 0 && current_sign != prev_sign_)
            {
                oscillation_count_++;
            }

            if (current_sign != 0)
            {
                prev_sign_ = current_sign;
            }
        }

        // --- UNIFIED PARSER FORMAT ---
        RCLCPP_INFO(
            this->get_logger(),
            "err=%.3f | osc_count=%d | max_osc=%.3f | time=%.3f ms | heading_err=%.3f",
            err_abs,
            oscillation_count_,
            max_oscillation_,
            loop_time_ms,
            heading_err_abs
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ACCFollow>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}