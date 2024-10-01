



//int N =9;


#include "mhe_acado.cpp"
#include "mhe_acado_node.h"
#include "mhe_acado.h"

/**
 * @file   nmhe_fxfyfzlearning_main.cpp
 * @author Mohit Mehndiratta
 * @date   September 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;


class NMHE : public rclcpp::Node {
public:
    NMHE() : Node("nmhe_fxfyfzlearning") {
        // Initialize subscribers
        estimation_on_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "regression_on", 1, std::bind(&NMHE::estimation_on_cb, this, std::placeholders::_1));
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mobula/rov/odometry", 1, std::bind(&NMHE::vel_cb, this, std::placeholders::_1));
        nmpc_cmd_wrench_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "/mobula/rov/wrench", 1, std::bind(&NMHE::nmpc_cmd_wrench_cb, this, std::placeholders::_1));
        
        // Initialize publishers
        nmhe_predInit_pub_ = this->create_publisher<std_msgs::msg::Bool>("nmhe_learning/predInit", 1);
        nmhe_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nmhe_learning/uvw", 1);
        nmhe_dist_Fx_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nmhe_learning/Fx", 1);
        nmhe_dist_Fy_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nmhe_learning/Fy", 1);
        nmhe_dist_Fz_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nmhe_learning/Fz", 1);
        nmhe_exeTime_pub_ = this->create_publisher<std_msgs::msg::Float64>("nmhe_learning/exeTime", 1);
        nmhe_kkt_pub_ = this->create_publisher<std_msgs::msg::Float64>("nmhe_learning/kkt", 1);
        nmhe_obj_pub_ = this->create_publisher<std_msgs::msg::Float64>("nmhe_learning/obj", 1);

        rclcpp::Rate rate(1.0 / sampleTime_);
    }

    void publish_uvw_FxFyFz( struct estimation_struct &estimationstruct) {
        // Publish data
        auto predInit_msg = std_msgs::msg::Bool();
        predInit_msg.data = is_prediction_init_;
        nmhe_predInit_pub_->publish(predInit_msg);

        auto uvw_vec_msg = std_msgs::msg::Float64MultiArray();
        uvw_vec_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        uvw_vec_msg.layout.dim[0].size = uvw_vec_.size();
        uvw_vec_msg.layout.dim[0].stride = 1;
        uvw_vec_msg.layout.dim[0].label = "u_vel, v_vel, w_vel (m/sec)";
        uvw_vec_msg.data = uvw_vec_;
        nmhe_vel_pub_->publish(uvw_vec_msg);

        // Publish force disturbances and other metrics...
    }

private:
   // void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    //    current_state_msg_ = *msg;
    //}

    void estimation_on_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        estimation_on_msg_ = *msg;
    }

    void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        current_vel_rate_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                             msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    }

    void nmpc_cmd_wrench_cb(const geometry_msgs::msg::Wrench::SharedPtr msg) {
       

        nmpc_cmd_input_(0) = msg->force.x;
        nmpc_cmd_input_(1) = msg->force.y;
        nmpc_cmd_input_(2) = msg->force.z;

        
    }



    // ROS 2 subscriptions and publishers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estimation_on_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Wrench>::SharedPtr nmpc_cmd_wrench_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nmhe_predInit_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nmhe_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nmhe_dist_Fx_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nmhe_dist_Fy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nmhe_dist_Fz_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nmhe_exeTime_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nmhe_kkt_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nmhe_obj_pub_;

    //mavros_msgs::msg::State current_state_msg_;
    std_msgs::msg::Bool estimation_on_msg_;

    Eigen::VectorXd current_vel_rate_;
    Eigen::VectorXd nmpc_cmd_wrench_;
   // Eigen::VectorXd nmpc_cmd_Fz_;

    bool is_prediction_init_;
    std::string mocap_topic_part_;
    double sampleTime_ = 0.03;
    static constexpr double NMPC_N = 30;

    std::vector<double> uvw_vec_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NMHE>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



