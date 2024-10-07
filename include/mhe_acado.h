#ifndef MHE_ACADO_H
#define MHE_ACADO_H


#include <math.h>
#include <eigen3/Eigen/Dense>
#include <time.h>
#include "nmhe_common.h"
#include "nmhe_auxiliary_functions.h"
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <vector>
#include <iostream>
extern NMHEworkspace nmheWorkspace;
extern NMHEvariables nmheVariables;


struct nmhe_struct_
{
	bool verbose;

	Eigen::VectorXd X0;
	Eigen::VectorXd W;
	Eigen::VectorXd WN;
	Eigen::VectorXd process_noise_cov;
	Eigen::VectorXd SAC;
	Eigen::VectorXd xAC;
	double sample_time;

};

class NMHE_FXFYFZ
{
	private:
		
		bool is_estimator_init, is_prediction_init;


		Eigen::MatrixXd WL_mat;

		int run_cnt;

	public:
		int acado_feedbackStep_fb;

		nmhe_struct_ nmhe_inp_struct;


		struct acado_struct
		{
			boost::function<int(void)> initializeSolver;
			boost::function<int(void)> preparationStep;
			boost::function<int(void)> feedbackStep;
			boost::function<real_t(void)> getKKT;
			boost::function<real_t(void)> getObjective;
			boost::function<void(void)> printDifferentialVariables;
			boost::function<void(void)> printControlVariables;
			boost::function<int(int)> updateArrivalCost;

			int acado_N;
			int acado_NX;
			int acado_NY;
			int acado_NYN;
			int acado_NU;
			int acado_NOD;

			real_t * x;
			real_t * u;
			real_t * od;
			real_t * y;
			real_t * yN;
			real_t * W;
			real_t * WN;
			real_t * SAC;
			real_t * xAC;
			real_t * WL;
		} nmhe_struct;

		struct estimation_struct
		{
			double u_est;
			double v_est;
			double w_est;
			double Fx_dist_est;
			double Fy_dist_est;
			double Fz_dist_est;
			double exe_time;
			double kkt_tol;
			double obj_val;

		} nmhe_est_struct;

		NMHE_FXFYFZ(struct nmhe_struct_ &_nmhe_inp_struct);
		~NMHE_FXFYFZ();

		bool return_estimator_init_value();

		void nmhe_init(struct acado_struct &acadostruct);

		void nmhe_core(struct acado_struct &acadostruct, struct estimation_struct &estimationstruct, Eigen::VectorXd &velocity, Eigen::VectorXd &control);

		//void publish_uvw_FxFyFz(struct estimation_struct &estimationstruct);

	protected:
		//ros::NodeHandle private_nh;

		void set_measurements(struct acado_struct &acadostruct, Eigen::VectorXd &currentvelrates, Eigen::Vector3d &nmpccmd);

};

#endif  // MHE_ACADO_H
