/**
*    \author Hakim Amer
*    \date   1/10/2024
*/

#include <acado_code_generation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>

int main( )
{
    USING_NAMESPACE_ACADO
//    using namespace Eigen;

//    using Eigen::Matrix3d;

    // Variables:
    DifferentialState u;                // the translation velocity along X_B
    DifferentialState v;                // the translation velocity along Y_B
    DifferentialState w;                // the translation velocity along Z_B
    DifferentialState Fx_dist;          // the external force along X_B
    DifferentialState Fy_dist;          // the external force along Y_B
    DifferentialState Fz_dist;          // the external force along Z_B

    OnlineData p_rate;                  // the roll rate
    OnlineData q_rate;                  // the pitch rate
    OnlineData r_rate;                  // the yaw rate

    Control phi;                        // the roll angle
    Control theta;                      // the pitch angle
    Control Fz;                         // the external force along Z_B

    const double m = 11.4;               // kg
    const double g = 9.81;              // m/s^2

    // Model equations:
    DifferentialEquation f;

    f << dot(u) == r_rate*v - q_rate*w + g*sin(theta) + Fx_dist;
    f << dot(v) == p_rate*w - r_rate*u - g*sin(phi)*cos(theta) + Fy_dist;
    f << dot(w) == q_rate*u - p_rate*v - g*cos(phi)*cos(theta) + (1/m)*Fz + Fz_dist;
    f << dot(Fx_dist) == 0;
    f << dot(Fy_dist) == 0;
    f << dot(Fz_dist) == 0;

    // Reference functions and weighting matrices:
    Function h, hN;
    h << u << v << w << phi << theta << Fz;
    hN << u << v << w;

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

    ocp.subjectTo( -6 <= Fx_dist <= 6);
    ocp.subjectTo( -6 <= Fy_dist <= 6);
    ocp.subjectTo( -6 <= Fz_dist <= 6);

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

    std::string path = ros::package::getPath("acado_ccode_generation");
    std::string path_dir = path + "/solver/NMHE_FxFyFzlearning";
    ROS_INFO("%s", path_dir.c_str());

    try
    {
        ROS_WARN("TRYING TO EXPORT");
        if (mhe.exportCode(path_dir) != SUCCESSFUL_RETURN) ROS_ERROR("FAIL EXPORT CODE");
    }
    catch (...)
    {
        ROS_ERROR("FAIL TO EXPORT");
    }

    mhe.printDimensionsQP();

    ROS_WARN("DONE CCODE");

    return EXIT_SUCCESS;
}