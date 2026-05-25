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
#include <random>

using std::placeholders::_1;

class PSO_Follow : public rclcpp::Node
{
public:
    PSO_Follow()
    : Node("pso_follow"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        d_ref_ = 0.7;
        stop_threshold_ = 0.1;

        v_max_ = 0.6;
        w_max_ = 3;

        alpha_ = 2.0;
        beta_ = 1.5;

        swarm_size_ = 20;
        max_iter_ = 50;

        dt_ = 0.3;
        b_ = 1.0;

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

        w_inertia_ = 0.72;
        c1_ = 1.5;
        c2_ = 1.5;

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PSO_Follow::scan_callback, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PSO_Follow::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "PSO follower with heading error metric started");
    }

private:
    // ===== params =====
    double d_ref_, stop_threshold_;
    double v_max_, w_max_;
    double alpha_, beta_;

    int swarm_size_, max_iter_;

    double dt_, b_;
    double max_accel_, max_delta_w_;
    double predict_time_, dt_sim_;
    double v_reso_, w_reso_;
    double robot_radius_;

    // ===== Metrics and Oscillation Tracking Variables =====
    bool reached_setpoint_ = false;
    double zero_band_ = 0.02;
    int oscillation_count_ = 0;
    double max_oscillation_ = 0.0;
    int prev_sign_ = 0;

    double prev_v_, prev_w_, smooth_gain_;

    double w_inertia_, c1_, c2_;

    // ===== PSO state =====
    std::vector<std::vector<double>> particles_;
    std::vector<std::vector<double>> velocities_;
    std::vector<std::vector<double>> pbest_;
    std::vector<double> pbest_score_;

    std::vector<double> gbest_;
    double gbest_score_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::mt19937 gen_{std::random_device{}()};

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = msg;
    }

    std::pair<double,double> predict_relative_state(double dx, double dy, double v, double w)
    {
        double x_rel = dx - v * dt_;
        double y_rel = dy;

        double th = -w * dt_;
        double c = std::cos(th);
        double s = std::sin(th);

        double dx_next = c * x_rel - s * y_rel;
        double dy_next = s * x_rel + c * y_rel;

        return {dx_next, dy_next};
    }

    double fitness(const std::vector<double>& X, double dx, double dy)
    {
        double v = X[0];
        double w = X[1];

        auto [dx_p, dy_p] = predict_relative_state(dx, dy, v, w);

        double d = std::hypot(dx_p, dy_p);
        double theta = std::atan2(dy_p, dx_p);

        double e_d = std::abs(d - d_ref_);
        double e_theta = std::abs(theta);

        double effort = 0.05 * std::abs(v) + 0.02 * std::abs(w);

        return alpha_ * e_d + beta_ * e_theta + effort;
    }

    void init_pso()
    {
        particles_.resize(swarm_size_, std::vector<double>(2));
        velocities_.resize(swarm_size_, std::vector<double>(2, 0.0));
        pbest_.resize(swarm_size_, std::vector<double>(2));
        pbest_score_.resize(swarm_size_);

        std::uniform_real_distribution<double> vdist(-v_max_, v_max_);
        std::uniform_real_distribution<double> wdist(-w_max_, w_max_);

        for (int i = 0; i < swarm_size_; i++)
        {
            particles_[i][0] = vdist(gen_);
            particles_[i][1] = wdist(gen_);

            pbest_[i] = particles_[i];
            pbest_score_[i] = std::numeric_limits<double>::infinity();
        }
    }

    std::pair<double,double> pso_optimize(double dx, double dy)
    {
        init_pso();

        std::uniform_real_distribution<double> r01(0.0, 1.0);

        for (int iter = 0; iter < max_iter_; iter++)
        {
            gbest_score_ = std::numeric_limits<double>::infinity();
            gbest_.assign(2, 0.0);

            for (int i = 0; i < swarm_size_; i++)
            {
                double fit = fitness(particles_[i], dx, dy);

                if (fit < pbest_score_[i])
                {
                    pbest_score_[i] = fit;
                    pbest_[i] = particles_[i];
                }

                if (fit < gbest_score_)
                {
                    gbest_score_ = fit;
                    gbest_ = particles_[i];
                }
            }

            for (int i = 0; i < swarm_size_; i++)
            {
                for (int d = 0; d < 2; d++)
                {
                    double r1 = r01(gen_);
                    double r2 = r01(gen_);

                    velocities_[i][d] =
                        w_inertia_ * velocities_[i][d]
                        + c1_ * r1 * (pbest_[i][d] - particles_[i][d])
                        + c2_ * r2 * (gbest_[d] - particles_[i][d]);

                    particles_[i][d] += velocities_[i][d];
                }

                particles_[i][0] = std::clamp(particles_[i][0], -v_max_, v_max_);
                particles_[i][1] = std::clamp(particles_[i][1], -w_max_, w_max_);
            }
        }

        return {gbest_[0], gbest_[1]};
    }

    std::pair<double,double> dwa_safety_filter(double v_cmd, double w_cmd)
    {
        if (!scan_)
            return {v_cmd, w_cmd};

        double min_v = std::max(-v_max_, v_cmd - max_accel_ * dt_sim_);
        double max_v = std::min(v_max_,  v_cmd + max_accel_ * dt_sim_);

        double angle_expand = (std::abs(v_cmd) > 0.05) ? 0.7 : 0.4;

        double min_w = std::max(-w_max_, w_cmd - max_delta_w_ * dt_sim_ - angle_expand);
        double max_w = std::min(w_max_,  w_cmd + max_delta_w_ * dt_sim_ + angle_expand);

        double best_v = 0.0, best_w = 0.0;
        double min_cost = std::numeric_limits<double>::infinity();
        bool found = false;

        int n_v = static_cast<int>(std::floor((max_v - min_v) / v_reso_));
        int n_w = static_cast<int>(std::floor((max_w - min_w) / w_reso_));

        for (int i = 0; i <= n_v + 1; ++i)
        {
            double v = min_v + i * v_reso_;
            for (int j = 0; j <= n_w + 1; ++j)
            {
                double w = min_w + j * w_reso_;

                if (std::isinf(v) || std::isinf(w))
                    continue;

                if (std::isnan(v) || std::isnan(w))
                    continue;

                auto traj = predict_trajectory(v, w);
                double obstacle_cost = calc_obstacle_cost(traj);

                if (std::isinf(obstacle_cost))
                    continue;

                double tracking_cost = std::abs(v - v_cmd) + std::abs(w - w_cmd);
                double final_cost = 3.5 * obstacle_cost + 4 * tracking_cost;

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

    std::vector<std::pair<double,double>> predict_trajectory(double v, double w)
    {   
        std::vector<std::pair<double,double>> traj;
        double x=0,y=0,yaw=0;

        for(double t=0;t<=predict_time_;t+=dt_sim_)
        {
            x+=v*std::cos(yaw)*dt_sim_;
            y+=v*std::sin(yaw)*dt_sim_;
            yaw+=w*dt_sim_;
            traj.emplace_back(x,y);
        }
        return traj;
    }

    double calc_obstacle_cost(const std::vector<std::pair<double,double>>& traj)
    {
        if (!scan_) return 0.0;

        double min_dist = std::numeric_limits<double>::infinity();

        for(auto&p:traj)
        {
            for(size_t i=0; i<scan_->ranges.size(); ++i)
            {
                double r=scan_->ranges[i];
                if(std::isinf(r)||std::isnan(r)) continue;

                double angle=scan_->angle_min+i*scan_->angle_increment;

                double obs_x=r*std::cos(angle);
                double obs_y=r*std::sin(angle);

                double dist=std::hypot(p.first-obs_x,p.second-obs_y);

                min_dist = std::min(min_dist, dist);
            }
        }

        if(min_dist < robot_radius_)
            return std::numeric_limits<double>::infinity();

        return 1.0/(min_dist+0.01);
    }

    int sign_of(double x)
    {
        if (x > zero_band_) return 1;
        if (x < -zero_band_) return -1;
        return 0;
    }

    void control_loop()
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        geometry_msgs::msg::TransformStamped trans;

        try
        {
            trans = tf_buffer_.lookupTransform("base_link","leader/base_link",tf2::TimePointZero);
        }
        catch(...)
        {
            return;
        }

        double dx = trans.transform.translation.x;
        double dy = trans.transform.translation.y;

        double distance = std::hypot(dx, dy);
        double heading = std::atan2(dy, dx);

        double v_cmd, w_cmd;

        if (std::abs(distance - d_ref_) < stop_threshold_ && std::abs(heading) < 0.08)
        {
            v_cmd = 0.0;
            w_cmd = 0.0;
        }
        else
        {
            auto res = pso_optimize(dx, dy);
            v_cmd = res.first;
            w_cmd = res.second;
        }

        auto filtered = dwa_safety_filter(v_cmd, w_cmd);

        double v = (1.0 - smooth_gain_) * filtered.first + smooth_gain_ * prev_v_;
        double w = (1.0 - smooth_gain_) * filtered.second + smooth_gain_ * prev_w_;

        prev_v_ = v;
        prev_w_ = w;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(v, -v_max_, v_max_);
        cmd.angular.z = std::clamp(w, -w_max_, w_max_);

        cmd_pub_->publish(cmd);

        auto loop_end = std::chrono::high_resolution_clock::now();
        double loop_time_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();

        double error = distance - d_ref_;
        double err_abs = std::abs(error);
        double heading_err_abs = std::abs(heading);

        // --- Start detection after reaching setpoint ---
        if (!reached_setpoint_ && std::abs(error) < zero_band_)
        {
            reached_setpoint_ = true;
            prev_sign_ = 0;
        }

        // --- Oscillation Tracking Logic ---
        if (reached_setpoint_)
        {
            int current_sign = sign_of(error);

            if (current_sign != 0)
            {
                if (std::abs(error) > max_oscillation_)
                {
                    max_oscillation_ = std::abs(error);
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

        // --- MATCHES ALL OTHER LOGS ---
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
    rclcpp::spin(std::make_shared<PSO_Follow>());
    rclcpp::shutdown();
    return 0;
}