/**
 * @file   Nonlinear Moving Horizon Disturbance Observer
 * @brief  Implementation based on Mohit Mehndiratta's code
 *         (https://github.com/mohit004/wind_nmhe_regression/tree/master)
 * @date   October 1, 2024
 * @author Hakim Amer
 * @contact Email: aha@eiva.com
 */


#include "mhe_acado.cpp"
#include "mhe_acado_node.h"
#include "mhe_acado.h"

// ros2 
#include <memory>
#include <vector>
#include <cmath>


#define SAMPLE_TIME 20ms



void NMHENode::publish_uvw_FxFyFz( NMHE_FXFYFZ::estimation_struct& estimationstruct) {
    // Publish data
    auto predInit_msg = std_msgs::msg::Bool();

       //std::cout << "Publishing disturbance prediction..." << std::endl;
   std::cout<<estimationstruct.u_est;
    std::vector<double> uvw_vec = {estimationstruct.u_est, estimationstruct.v_est, estimationstruct.w_est};


    std::vector<double> F_dist_vec = {estimationstruct.Fx_dist_est, estimationstruct.Fy_dist_est, estimationstruct.Fz_dist_est};

    auto uvw_vec_msg = std_msgs::msg::Float64MultiArray();
    uvw_vec_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    uvw_vec_msg.layout.dim[0].size = uvw_vec.size();
    uvw_vec_msg.layout.dim[0].stride = 1;
    uvw_vec_msg.layout.dim[0].label = "u_vel, v_vel, w_vel (m/sec)";
    uvw_vec_msg.data = uvw_vec;


    auto dist_vec_msg = std_msgs::msg::Float64MultiArray();
    dist_vec_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    dist_vec_msg.layout.dim[0].size = F_dist_vec.size();
    dist_vec_msg.layout.dim[0].stride = 1;
    dist_vec_msg.layout.dim[0].label = "F_x, F_y, F_z ";

    std::cout <<"Fx_dist_est: " << estimationstruct.Fx_dist_est << std::endl;
    dist_vec_msg.data = F_dist_vec;

    nmhe_vel_pub_->publish(uvw_vec_msg);


    // Publish force disturbances and other metrics...
    nmhe_dist_F_dist_pub_->publish(dist_vec_msg);

    


}



   void  NMHENode::state_cb(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

        double roll = 0, pitch = 0, yaw = 0;

       attitude_quat_ = {
           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};

         velocity_ << msg->twist.twist.linear.x, 
                      msg->twist.twist.linear.y, 
                     msg->twist.twist.linear.z, 
                    msg->twist.twist.angular.x, 
                    msg->twist.twist.angular.y, 
                     msg->twist.twist.angular.z;

         pose_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw};

  }



    void NMHENode::control_cb(const geometry_msgs::msg::Wrench::ConstSharedPtr msg) {

           control_input_ << msg->force.x, msg->force.y, msg->force.z;        
        std::cout<<"control X: "<< msg->force.x<<std::endl;


    }


 void NMHENode::estimation_on_cb(const std_msgs::msg::Bool::ConstSharedPtr msg) {
       estimation_on_msg = *msg;
    }


  void NMHENode::main_loop() {



        if (estimation_on_msg.data ) {
            RCLCPP_INFO(this->get_logger(), "Estimation switch turned on!");
            print_flag_estimation_on = false;  // Ensure message prints only once
        }    

        // Call the NMHE core function
        std::cout<<"control X: "<< control_input_<<std::endl;
        std::cout<<"velocity X: "<< velocity_<<std::endl;

        nmhe->nmhe_core(nmhe->nmhe_struct, nmhe->nmhe_est_struct, velocity_, control_input_);

        // Check for feedback step failure
        if (nmhe->acado_feedbackStep_fb != 0) {
            estimator_stop = true;
        }

    
        // Publish results
        this->publish_uvw_FxFyFz(nmhe->nmhe_est_struct);
    }



NMHENode::NMHENode() : Node("nmhe_node")
{
    // ---------------
    // Main loop timer
    // ---------------


    this-> main_loop_timer = this->create_wall_timer(SAMPLE_TIME, std::bind(&NMHENode::main_loop, this));

    // Initialize subscribers
    this->estimation_on_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "regression_on", 1, std::bind(&NMHENode::estimation_on_cb, this, std::placeholders::_1));    // flag to activate/deactivate regression 

    this->state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mobula/rov/odometry", 1, std::bind(&NMHENode::state_cb, this, std::placeholders::_1));     // state feedback subscriber (from simulation/real-robot)

    this->control_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
       "/mobula/rov/wrench", 1, std::bind(&NMHENode::control_cb, this, std::placeholders::_1));     // control input subscriber (from MPC)

    //this->nmhe = new NMHE_FXFYFZ(this->nmhe_struct);     // create a new NMHE structure
    control_input_.resize(3);
    velocity_.resize(6);

     control_input_ << 0.0, 0.0, 0.0;   
     velocity_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;   

    // Initialize publishers
    this->nmhe_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nmhe_learning/uvw", 1);
    this->nmhe_dist_F_dist_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("nmhe_learning/dist", 1);


    // mhe parameters
    this->nmhe_struct.X0.resize(NMHE_NX);
    this->nmhe_struct.W.resize(NMHE_NY);
    this->nmhe_struct.WN.resize(NMHE_NYN);
    this->nmhe_struct.process_noise_cov.resize(NMHE_NX);
    this->nmhe_struct.SAC.resize(NMHE_NX);
    this->nmhe_struct.xAC.resize(NMHE_NX);

    this->nmhe_struct.X0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // initial state
    //this->nmhe_struct.W << 10.0, 10.0, 0.02, 0.2, 0.02, 0.02;
    
    
    this->nmhe_struct.W <<  0.003, 0.003, 0.003, 0.003, 0.003, 0.003; // weights

    this->nmhe_struct.WN << 0.003, 0.0003, 0.0003;  // terminal weights
    
    this->nmhe_struct.process_noise_cov << 3.0, 3.0, 3.0, 1.0, 1.0, 1.0;  // process noise covariance

    //this->nmhe_struct.process_noise_cov << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;  // process noise covariance
    
    
    this->nmhe_struct.SAC << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3; // SAC values

    //this->nmhe_struct.SAC << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // SAC values

    this->nmhe_struct.xAC << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

     this->nmhe = new NMHE_FXFYFZ(this->nmhe_struct); 

      if (!nmhe->return_estimator_init_value())
        {
            nmhe->nmhe_init(nmhe->nmhe_struct);
            if (!nmhe_struct.verbose && nmhe->return_estimator_init_value())
            {
                std::cout<<"**********************************\n";
                std::cout<<"NMHE_FXFYFZ: initialized correctly\n";
                std::cout<<"**********************************\n";
            }
        }


}




int main(int argc, char **argv) {
    std::cout << "Starting NMHE Node..." << std::endl;

    rclcpp::init(argc, argv);

    auto mhe_node = std::make_shared<NMHENode>();

    rclcpp::spin(mhe_node);

    rclcpp::shutdown();

    return 0;
}







