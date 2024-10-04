/**
*    \author Mohit Mehndiratta
*    \date   2019
*/

#include <acado_code_generation.hpp>

#include <boost/algorithm/string.hpp>
#include <iostream>



int main( )
{

        //const double F_bouy = 114.8; // Bouyancy force (N)
    const double m = 13.4;    // BlueROV2 mass (kg)  
    const double g = 9.82;  // gravitational field strength (m/s^2)

    const double F_bouy = 1026 * 0.0115 * g; // Bouyancy force (N)
    const double eps = 0.00001;
    //const double F_bouy = 114.8; // Buoyancy force (N)

    const double X_ud = -2.6 ; // Added mass in x direction (kg)
    const double Y_vd = -18.5 ; // Added mass in y direction (kg)
    const double Z_wd = -13.3 ; // Added mass in z direction (kg)
    const double N_rd = -0.28 ; // Added mass for rotation about z direction (kg)

    const double I_xx = 0.21 ; // Moment of inertia (kg.m^2)
    const double I_yy = 0.245 ; // Moment of inertia (kg.m^2)
    const double I_zz = 0.245 ; // Moment of inertia (kg.m^2)

    const double X_u = -0.09 ; // Linear damping coefficient in x direction (N.s/m)
    const double Y_v  = -0.26 ; // Linear damping coefficient  in y direction (N.s/m)
    const double Z_w = -0.19; // Linear damping coefficient  in z direction (N.s/m)
    const double N_r = -4.64 ;  // Linear damping coefficient for rotation about z direction (N.s/rad)

    const double X_uc = -34.96 ; // quadratic damping coefficient in x direction (N.s^2/m^2)
    const double Y_vc = -103.25 ; // quadratic damping coefficient  in y direction (N.s^2/m^2)
    const double Z_wc = -74.23 ; // quadratic damping coefficient  in z direction (N.s^2/m^2)
    const double N_rc = - 0.43 ; // quadratic damping coefficient for rotation about z direction (N.s^2/rad^2)

    USING_NAMESPACE_ACADO
    std::string path ="/home/hakim/ros2/src/mhe_acado_ros2";

    //State Variables:
    //DifferentialState x;  // the body position w.r.t X_I
    //DifferentialState y;  // the body position w.r.t Y_I
    //DifferentialState z;  // the body position w.r.t Z_I


	DifferentialState u;                // the translation velocity along X_B
    DifferentialState v;                // the translation velocity along Y_B
    DifferentialState w;                // the translation velocity along Z_B

    DifferentialState Fx_dist;          // the external force along X_B
    DifferentialState Fy_dist;          // the external force along Y_B
    DifferentialState Fz_dist;          // the external force along Z_B

  
    
    //DifferentialState psi;  // yaw angle 
    OnlineData r;   // yaw rate

   // OnlineData Fx_dist;  // the external disturbance force along X_B
   // OnlineData Fy_dist;  // the external disturbance force along Y_B
   // OnlineData Fz_dist;  // the external disturbance force along Z_B
    // MPC control input 
    Control X;  // Force along X_B
    Control Y;  // Force along Y_B
    Control Z;  // Force along Z_B
    Control M_z;  // Torque about Z_B (Yawing moment)

    // BlueROV2 Model Parameters 
    

    // Model equations: 2-D model, assuming no roll or pitch
    DifferentialEquation f;

    //f << dot(x) == cos(psi) * u - sin(psi) * v;
    //f << dot(y) == sin(psi) * u +  cos(psi) * v;
    //f << dot(z) ==  w;
 
    f << dot(u) == (X + (m * v - Y_vd * v) * r + (X_u + X_uc *sqrt( u * u + eps) ) * u)/(m - X_ud) + Fx_dist ;
    f << dot(v) == (Y - (m * u - X_ud * u) * r + (Y_v + Y_vc *sqrt( v * v + eps) ) * v)/(m - Y_vd) + Fy_dist ;
    f << dot(w) == (Z + (Z_w + Z_wc * sqrt(w * w + eps)) * w + (m * g - F_bouy))/(m - Z_wd) + Fz_dist ;

    //f << dot(psi) ==  r;
    //f << dot(r) == (M_z - (m * v - Y_vd * v) * u - (X_ud * u - m * u) * v + (N_r + N_rc * sqrt(r * r + eps)) * r)/(I_zz - N_rd);
    

	f << dot(Fx_dist) == 0;
    f << dot(Fy_dist) == 0;
    f << dot(Fz_dist) == 0;

    // Reference functions and weighting matrices:
    Function h, hN;
   // h << x << y << z << u << v << w << psi << r << X << Y<< Z << M_z;
   // hN << x << y << z << u << v << w << psi << r;


    h  << u << v << w  << X << Y<< Z << M_z;
    hN  << u << v << w ;



    BMatrix W = eye<bool>( h.getDim() );
    BMatrix WN = eye<bool>( hN.getDim() );

    //
    // Optimal Control Problem
    //
    double N = 40;
    double Ts = 0.01;
    OCP ocp(0.0, N*Ts, N);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo( -10 <= Fx_dist <= 10);
    ocp.subjectTo( -10  <= Fy_dist <= 10);
    ocp.subjectTo( -10  <= Fz_dist <= 10);

    // Export the code:
    OCPexport mhe( ocp );

    mhe.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    mhe.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    mhe.set( INTEGRATOR_TYPE, INT_RK4 );
    mhe.set(NUM_INTEGRATOR_STEPS, 2 * N);

    // NOTE: Those three options define MHE configuration!
    mhe.set( FIX_INITIAL_STATE, NO );
    mhe.set( SPARSE_QP_SOLUTION, CONDENSING );
    mhe.set( QP_SOLVER, QP_QPOASES );
    mhe.set( HOTSTART_QP, NO );

    mhe.set(CG_HARDCODE_CONSTRAINT_VALUES, YES); // Possible to Change Constraints Afterwards (only with qpOASES)
//    mhe.set( LEVENBERG_MARQUARDT, 1.0e-4 );

    mhe.set( GENERATE_TEST_FILE, NO );
    mhe.set( GENERATE_MAKE_FILE, NO );
    mhe.set( GENERATE_MATLAB_INTERFACE, NO );
    mhe.set( GENERATE_SIMULINK_INTERFACE, NO );
    mhe.set(CG_USE_ARRIVAL_COST, YES);

    mhe.set( PRINTLEVEL, DEBUG);

    // Optionally set custom module name:
    mhe.set( CG_MODULE_NAME, "nmhe" );
    mhe.set( CG_MODULE_PREFIX, "NMHE" );

    //std::string path = ros::package::getPath("acado_ccode_generation");
    //std::string path_dir = path + "/solver/NMHE_FxFyFzlearning";
    //ROS_INFO("%s", path_dir.c_str());

    //try
   // {
    //    ROS_WARN("TRYING TO EXPORT");
    //    if (mhe.exportCode(path_dir) != SUCCESSFUL_RETURN) ROS_ERROR("FAIL EXPORT CODE");
   // }
   // catch (...)
    //{
    //    ROS_ERROR("FAIL TO EXPORT");
    //}

    if (mhe.exportCode( path + "/model/codegen" ) != SUCCESSFUL_RETURN)
  		exit( EXIT_FAILURE );

    mhe.printDimensionsQP();

    //ROS_WARN("DONE CCODE");

    return EXIT_SUCCESS;
}