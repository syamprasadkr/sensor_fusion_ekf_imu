// The header files
#include <iostream>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
#include "kf.h"

// Constructor
Kalman_Filter::Kalman_Filter()
{
	this -> state_init();
	this -> covar_init();
	this -> obs_init();
	this -> motion_model_init();
	this -> g_jacobian_init();
	this -> q_noise_init();
	this -> obs_model_init();
	this -> r_noise_init();	
}

// Initailize the Kalman Filter state vector
void Kalman_Filter::state_init()
{
	x_state << init_val, init_val, init_val;
}

// Initailize the Kalman Filter covariance matrix
void Kalman_Filter::covar_init()
{
	p_covariance(thetax_idx, thetax_idx) = initial_uncertainity;
	p_covariance(thetay_idx, thetay_idx) = initial_uncertainity;
	p_covariance(thetaz_idx, thetaz_idx) = initial_uncertainity;	
}

// Initailize the motion model matrix
void Kalman_Filter::motion_model_init()
{
	g_motion_model << init_val, init_val, init_val;
}

// Initailize the matrix for jacobian of motion model
void Kalman_Filter::g_jacobian_init()
{
	gj_motion_jacobian = Eigen::MatrixXd::Identity(gj_motion_jacobian.rows(), gj_motion_jacobian.cols());
}

// Initailize the process noise matrix
void Kalman_Filter::q_noise_init()
{
	q_process_noise = Eigen::MatrixXd::Zero(vec_size, vec_size);
} 

// Initailize the actual observation vector
void Kalman_Filter::obs_init()
{
	z_observation << init_val, init_val, init_val;
}

// Initailize the matrix for observation model. Note that this matrix remains the same all the time. 
void Kalman_Filter::obs_model_init()
{
	h_obs_model = Eigen::MatrixXd::Identity(h_obs_model.rows(), h_obs_model.cols());
}

// Initailize the observation noise matrix
void Kalman_Filter::r_noise_init()
{
	r_obs_noise << noise_ax * noise_ax, init_val, init_val,
	init_val, noise_ay * noise_ay, init_val,
	init_val, init_val, noise_az * noise_az;
}

// Return state of the filter
Eigen::VectorXd Kalman_Filter::get_state() const
{
	return x_state;
}

// Return covariance matrix
Eigen::MatrixXd Kalman_Filter::get_covariance() const
{
	return p_covariance;
}

// Calculate and return the current prediction of observation
Eigen::VectorXd Kalman_Filter::get_predicted_obs() const
{
	return h_obs_model * x_state;
}

// Return the gyro angle rates in rad/ s
Eigen::VectorXd Kalman_Filter::get_control() const
{
	return u_control;
}

// Return the actual observation
Eigen::VectorXd Kalman_Filter::get_measurement() const
{
	return z_observation;
}

// Assign values to the state vector of the Kalman Filter
void Kalman_Filter::set_state(const Eigen::VectorXd state)
{
	x_state = state;
}

// Assign values to the control input vetcor
void Kalman_Filter::set_control(const Eigen::VectorXd u_ctrl)
{
	u_control = u_ctrl;
}

// Assign values to the actual observation vector
void Kalman_Filter::set_measurement(const Eigen::VectorXd z_obs)
{
	z_observation = z_obs;
}

// Calculate the motion model in each timestep
void Kalman_Filter::motion_model_calc(const double loop_time)
{
	double phi = x_state(thetax_idx);
	double theta = x_state(thetay_idx);
	// Conversion of gyro angle rates to Euler angle rates
	u_euler_control << u_control(thetax_idx) + (u_control(thetay_idx) * sin(phi) * tan(theta)) + (u_control(thetaz_idx) * cos(phi) * tan(theta)), 
	(u_control(thetay_idx) * cos(phi)) - (u_control(thetaz_idx) * sin(phi)),
	(u_control(thetay_idx) * sin(phi) / cos(theta)) + (u_control(thetaz_idx) * cos(phi) / cos(theta));
	// The complete motion model
	g_motion_model = x_state + (loop_time * u_euler_control);
}

// Calculate the Jacobian of motion model. This is done because the motion model is non-linear.
void Kalman_Filter::motion_jacobian_calc(const double loop_time)
{
	double phi = x_state(thetax_idx);
	double theta = x_state(thetay_idx);
	gj_motion_jacobian(thetax_idx, thetax_idx) = gj_const + loop_time * u_control(thetay_idx) * tan(theta) * cos(phi) - loop_time * u_control(thetaz_idx) * tan(theta) * sin(phi);
	gj_motion_jacobian(thetax_idx, thetay_idx) = loop_time * (u_control(thetay_idx) * sin(phi) +  u_control(thetaz_idx) * cos(phi)) / (cos(theta) * cos(theta));
	gj_motion_jacobian(thetay_idx, thetax_idx) = (-loop_time) * (u_control(thetay_idx) * sin(phi) + u_control(thetaz_idx) * cos(phi));
	gj_motion_jacobian(thetaz_idx, thetax_idx) = (loop_time / cos(theta)) * (u_control(thetay_idx) * cos(phi) - u_control(thetaz_idx) * sin(phi));
	gj_motion_jacobian(thetaz_idx, thetay_idx) = (loop_time * tan(theta) / cos(theta)) * (u_control(thetay_idx) * sin(phi) + u_control(thetaz_idx) * cos(phi));  
}

// The process noise matrix calculation for each time step
void Kalman_Filter::q_noise_calc(const double loop_time)
{
	double phi = x_state(thetax_idx);
	double theta = x_state(thetay_idx);
	Eigen::VectorXd stddev_euler{vec_size};
	// Conversion of gyro angle rate noise to Euler angle noise
	stddev_euler << noise_gx + (noise_gy * sin(phi) * tan(theta)) + (noise_gz * cos(phi) * tan(theta)), (noise_gy * cos(phi)) - (noise_gz * sin(phi)), (noise_gy * sin(phi) / cos(theta)) + (noise_gz * cos(phi) / cos(theta));
	stddev_euler = stddev_euler * loop_time;
	double mean = init_val;
	double stddev = stddev_euler.mean();
	//The updated noise values
	q_process_noise(thetax_idx, thetax_idx) = (stddev_euler(thetax_idx) * stddev_euler(thetax_idx));
	q_process_noise(thetay_idx, thetay_idx) = (stddev_euler(thetay_idx) * stddev_euler(thetay_idx));
	q_process_noise(thetaz_idx, thetaz_idx) = (stddev_euler(thetaz_idx) * stddev_euler(thetaz_idx));
} 

// Normalization of roll, pitch and yaw to keep them in the range -pi to pi radians.
void Kalman_Filter::normalize_xstate()
{
	normalize_negpi_to_pi(x_state(thetax_idx));
	normalize_negpi_to_pi(x_state(thetay_idx));
	normalize_negpi_to_pi(x_state(thetaz_idx));
	
}

// The prediction step of the Kalman Filter
void Kalman_Filter::prediction(const double loop_time)
{
	// The calculation of motion model, its jacobian and noise matrix. 
	this ->	motion_model_calc(loop_time);
	this -> motion_jacobian_calc(loop_time);
	this -> q_noise_calc(loop_time);
	// The prediction for state
	x_state = g_motion_model;
	// The prediction for covariance
	p_covariance = gj_motion_jacobian * p_covariance * gj_motion_jacobian.transpose() + q_process_noise; 
	// Normalization step
	normalize_xstate();

}

// The calculation of Kalman gain factor
void Kalman_Filter::kalman_gain_calc()
{
	k_gain = (p_covariance * h_obs_model.transpose()) * ((h_obs_model * p_covariance * h_obs_model.transpose() + r_obs_noise).inverse());
}

// The correction step of Kalman Filter
void Kalman_Filter::correction()
{
	// Obtain the predicted observation
	Eigen::VectorXd pred_obs = this -> get_predicted_obs();
	Eigen::VectorXd x1 = cos(z_observation.array());
	Eigen::VectorXd y1 = sin(z_observation.array());
	Eigen::VectorXd x2 = cos(pred_obs.array());
	Eigen::VectorXd y2 = sin(pred_obs.array());
	Eigen::VectorXd y = y1.array() * x2.array() - y2.array() * x1.array();
	Eigen::VectorXd x = x1.array() * x2.array() + y1.array() * y2.array();
	// Calculate the innovation or error between predicted observation and the actual observation
	Eigen::VectorXd innovation = y.binaryExpr(x, [](double y_comp, double x_comp) {return atan2(y_comp, x_comp);});
	// The correction for state
	x_state = x_state + (k_gain * innovation);
	// The correction for covariance
	p_covariance = (Eigen::MatrixXd::Identity(p_covariance.rows(), p_covariance.cols()) - k_gain * h_obs_model) * p_covariance;
	// The normalization step
	normalize_xstate();
}

// Print the different vectors and matrices used by Kalman Filter
void Kalman_Filter::print_kf() const
{
	std::cout << "State: " << std::endl << x_state << std::endl;
	std::cout << "Covariance Matrix: " << std::endl << p_covariance << std::endl;
	std::cout << "Observation: " << std::endl << z_observation << std::endl;
	std::cout << "Motion Model: " << std::endl << g_motion_model << std::endl;
	std::cout << "Process Noise: " << std::endl << q_process_noise << std::endl;
	std::cout << "Obs Model: " << std::endl << h_obs_model << std::endl;
	std::cout << "Obs Noise: " << std::endl << r_obs_noise << std::endl;
	std::cout << "Kalman Gain: " << std::endl << k_gain << std::endl;	
}

// Normalize an angle to the range -pi to pi
void normalize_negpi_to_pi(double& angle)
{
	angle = fmod(angle + M_PI, pi_twopi_factor * M_PI);
	if(angle < angle_cross)
	{
		angle += pi_twopi_factor * M_PI;
	}
	angle = angle - M_PI;
}

// Normalize an angle to the range 0 to 2pi
void normalize_zero_to_twopi(double& angle)
{
	angle = fmod(angle, pi_twopi_factor * M_PI);
	if(angle < angle_cross)
	{
		angle += pi_twopi_factor * M_PI;
	}
		
}


