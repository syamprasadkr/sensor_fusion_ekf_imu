#ifndef Kalman_Filter_H
#define Kalman_Filter_H

// General purpose 
constexpr size_t vec_size = 3;

// Class for Kalman Filter implementation
class Kalman_Filter
{
	private:
		Eigen::VectorXd x_state{vec_size};
		Eigen::MatrixXd p_covariance{vec_size, vec_size};
		Eigen::VectorXd u_control{vec_size};
		Eigen::VectorXd u_euler_control{vec_size};
		Eigen::VectorXd z_observation{vec_size};
		Eigen::VectorXd g_motion_model{vec_size};
		Eigen::MatrixXd gj_motion_jacobian{vec_size, vec_size};
		Eigen::MatrixXd q_process_noise{vec_size, vec_size};
		Eigen::MatrixXd h_obs_model{vec_size, vec_size};
		Eigen::MatrixXd r_obs_noise{vec_size, vec_size};
		Eigen::MatrixXd k_gain{vec_size, vec_size}; 
	public:
		Kalman_Filter();
		void state_init();
		void covar_init();
		void motion_model_init();
		void g_jacobian_init();
		void q_noise_init();
		void obs_init();
		void obs_model_init();
		void r_noise_init();
		Eigen::VectorXd get_state() const;
		Eigen::MatrixXd get_covariance() const;
		Eigen::VectorXd get_predicted_obs() const;
		Eigen::VectorXd get_control() const;
		Eigen::VectorXd get_measurement() const;
		void set_state(const Eigen::VectorXd);
		void set_control(const Eigen::VectorXd);
		void set_measurement(const Eigen::VectorXd);
		void motion_model_calc(const double);
		void motion_jacobian_calc(const double);
		void q_noise_calc(const double);
		void prediction(const double);
		void kalman_gain_calc();
		void correction();
		void normalize_xstate();	
		void print_kf() const;	
};

// General purpose
enum idx{thetax_idx = 0, thetay_idx, thetaz_idx};
// Initial uncertainity
constexpr double initial_uncertainity = 0.0;
// General purpose
constexpr int init_val = 0;
// To be used in Jacobian of motion model
constexpr int gj_const = 1;
// To be used in normalization functions
constexpr int angle_cross = 0;
constexpr int pi_twopi_factor = 2;
// The acceleration due to gravity in m/s^2
constexpr double g_acc = 9.8;
// The noise values of different sensors
constexpr double noise_gx = 0.0014;
constexpr double noise_gy = 0.0013;
constexpr double noise_gz = 0.0014;
constexpr double noise_ax = 0.0011;
constexpr double noise_ay = 0.0011;
constexpr double noise_az = 0.0011;
// To create the Gaussian noise matrices, if required.
constexpr std::array<std::array<int, vec_size>, vec_size> h1 = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
constexpr std::array<std::array<int, vec_size>, vec_size> h2 = {-1, -1, -1, 0, 0, 0, 1, 0, 1}; 
// Normlaize an angle to the range -pi to pi
void normalize_negpi_to_pi(double&);
// Normalize an angle to the range 0 to 2pi
void normalize_zero_to_twopi(double&);
#endif
