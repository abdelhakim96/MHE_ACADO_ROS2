#ifndef MHE_ACADO_NODE_H
#define MHE_ACADO_NODE_H


#define _USE_MATH_DEFINES

#include <memory>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <thread>
#include <chrono>
#include <functional>
#include <string>
#include <stdint.h>

// Eigen
#include <eigen3/Eigen/Dense>

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 messages
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

// PX4 messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

// Custom headers
#include "nmhe_common.h"
#include "nmhe_auxiliary_functions.h"
#include "mhe_acado.h"

// TF2
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
using namespace std::chrono_literals;  // Add this line

class NMHENode : public rclcpp::Node{   //NMHE node class inherts rclcpp node base class 

public: 
      NMHENode();   //default constructor
private:
    rclcpp::TimerBase::SharedPtr main_loop_timer;
    void main_loop();   //main loop that keeps running;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estimation_on_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr control_sub_;



    //Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nmhe_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nmhe_dist_Fx_pub_;


//    nmhe_dist_Fx_pub = nh.advertise<std_msgs::Float64MultiArray>("nmhe_learning/Fx", 1, true);

    //rclcpp::Publisher<NMHE_FXFYFZ::estimation_struct>::SharedPtr publish_uvw_FxFyFz;

   // callbacks

    void control_cb(const geometry_msgs::msg::Wrench::ConstSharedPtr msg);
    void state_cb(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void estimation_on_cb(const std_msgs::msg::Bool::ConstSharedPtr msg);

    // Functions for publishing
    //void publish_uvw_FxFyFz( struct estimation_struct& estimationstruct);
    //void publish_wrench(struct NMPC_PC::command_struct& commandstruct);


    void publish_uvw_FxFyFz(  NMHE_FXFYFZ::estimation_struct& estimationstruct);


    // State variables
    NMHE_FXFYFZ* nmhe;
    nmhe_struct_ nmhe_struct;
    //online_data_struct_ online_data;

    std::vector<double> attitude_quat_;
    Eigen::Vector3d control_input_;
    Eigen::VectorXd velocity_;
    std::vector<double> pose_;

    std::string mocap_topic_part;

    std_msgs::msg::Bool  estimation_on_msg;

    bool estimator_stop;

    double m_in, g_in;

    int print_flag_offboard = 1, print_flag_arm = 1, 
    print_flag_altctl = 1, print_flag_estimation_on = 0;

    double t, t_est_loop;

  
};

#endif  // MHE_ACADO_H

