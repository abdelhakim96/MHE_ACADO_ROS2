#ifndef MHE_ACADO_NODE_H
#define MHE_ACADO_NODE_H


#define _USE_MATH_DEFINES
#include <memory>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <fstream>
//#include "yaml-cpp/yaml.h"

#include "nmhe_common.h"
#include "nmhe_auxiliary_functions.h"
#include <thread>
#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "mhe_acado.h"


/* Some convenient definitions. */





using namespace std;
using namespace Eigen;
using std::placeholders::_1;

//extern "C"{
//__thread ACADOvariables acadoVariables;
//__thread ACADOworkspace acadoWorkspace;
//}




// odom topic /fmu/out/vehicle_odometry type:   px4_msgs/msg/VehicleOdometry

// control topic /fmu/in/vehicle_rates_setpoint  type:  px4_msgs/msg/VehicleRatesSetpoint
// TODO: check if veclocity is in body frame if not transform

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include <chrono>
#include <iostream>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

rclcpp::Time start_time;






nmhe_struct_ nmhe_struct;
std::string mocap_topic_part;

//mavros_msgs::State current_state_msg;
std_msgs::msg::Bool  estimation_on_msg;

bool estimator_stop;

double m_in, g_in;

int print_flag_offboard = 1, print_flag_arm = 1, 
print_flag_altctl = 1, print_flag_estimation_on = 0;

double t, t_est_loop;

Eigen::VectorXd current_vel_rate;
Eigen::Vector3d nmpc_cmd_ryp;
Eigen::Vector2d nmpc_cmd_Fz;


#endif  // MHE_ACADO_H

