/*

Name: Kalman Filter Sensor Fusion
Authors:
	Author1: Syamprasad K Rajagopalan
	Email1: syamprasad.rajagopalan@technicalproductsinc.us
	Email2: syamkrajagopalan@gmail.com
	Author2 (Supervisor): Ryan Mahoney
	Email1: ryan.mahoney@technicalproductsinc.us
Date: May 8, 2018

Description: This program accesses IMU data from sensors and runs a Kalman filter on the data to estimate the orientation of the sensor. The Gyroscope and the Accelerometer data are accessed from the IMU sensor unit integrated with a GARMIN VIRB 360 camera. The data are accessed via HTTP by posting JSON commands using cURL package. The gyroscope provides the data required for the process model of Kalman Filter while the accelerometer provides the data for the observation model. The Kalman gain value determines the trust factor between process and observation to produce an accurate estimate of the orientation. The results of the estimation are logged into a .csv file along with a few other parameters/ values that will help in analysis of the result. The tests undertaken so far, including still test, motion test and walking test, have produced very reliable results. The Kalman Filter successfully followed the trend indicated by the gyroscope while maintaining the average value indicated by the accelerometer.

Machine OS: Ubuntu 16.04.3 LTS (xenial)
Machine Architecture: x86_64

Files Required:
1) imu_data.h - Header file for handling data access from sensors.
2) imu_data.cpp - Implementation file for handling data access from sensors.
3) kalman_filter_imu.h - Header file for handling general functions, transfer of information between imu and kalman filter, and central coordination and control.
4) kalman_filter_imu.cpp - Implementation file for handling general functions, transfer of information between imu and kalman filter, and central coordination and control.
5) kf.h - Header file for Kalman filter.
6) kf.cpp - Implementation file for Kalman filter.
7) CmakeLists.txt - The cmake file to configure and generate Makefile required for building and running the code.

Build Instructions:
1) Make sure all the above files are available in a folder.
2) Open a terminal and type the following commands.
3) $cd /<path_to_folder>
4) $cmake .
5) $make

Executable File: kalman_filter_sf
Run Instructions:
1) Open a terminal and type the following commands.
2) $cd /<path_to_folder>
3) $./kalman_filter_sf
Output File: log_ornt_<Timestamp>.csv

*/

// The header files
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <curl/curl.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <Eigen/Dense>
#include "imu_data.h"
#include "kf.h"
#include "kalman_filter_imu.h"

// Returns the midpoint of two timestamps
boost::posix_time::ptime timestamp_midpoint(const boost::posix_time::ptime& t1, const boost::posix_time::ptime& t2)
{
	return t1 + (t2 - t1) / tstamp_divisor;
}

// Performs conversion  of raw accelerometer readings to roll, pitch and yaw observations
void acc_euler(const Eigen::VectorXd z_obs_reading, Eigen::VectorXd& z_obs, const double yaw_estimate = init_val)
{
	// Roll calculation
	z_obs(thetax_idx) = atan2(-z_obs_reading(thetay_idx), -z_obs_reading(thetaz_idx));
	// Pitch calculation
	z_obs(thetay_idx) = atan2((z_obs_reading(thetax_idx)), -(z_obs_reading(thetay_idx) * sin(z_obs(thetax_idx)) + z_obs_reading(thetaz_idx) * cos(z_obs(thetax_idx))));
	// The assignment of yaw estimate. See the comments in the main prior to the call for 'set_yaw' function.
	z_obs(thetaz_idx) = yaw_estimate;
}

// Log initial offsets, noise values, state and timestamp
void log_init(std::ofstream& log_file, const Eigen::VectorXd state_init, const boost::posix_time::ptime& prev_timestamp)
{
	log_file << "Accx_offset" << ","
		<< offset::accx_offset << std::endl
		<< "Accy_offset" << ","
		<< offset::accy_offset << std::endl
		<<  "Accz_offset" << ","
		<< offset::accz_offset << std::endl
		<<  "Gyrox_offset" << ","
		<< offset::gyrox_offset << std::endl
		<<  "Gyroy_offset" << ","
		<< offset::gyroy_offset << std::endl
		<<  "Gyroz_offset" << ","
		<< offset::gyroz_offset << std::endl
		<< "Noise_ax" << ","
		<< noise_ax << std::endl
		<< "Noise_ay" << ","
		<< noise_ay << std::endl
		<< "Noise_az" << ","
		<< noise_az << std::endl
		<< "Noise_gx" << ","
		<< noise_gx << std::endl
		<< "Noise_gy" << ","
		<< noise_gy << std::endl
		<< "Noise_gz" << ","
		<< noise_gz << std::endl
		<< "Initial_roll" << ","
		<< state_init(thetax_idx) << std::endl
		<< "Initial_pitch" << ","
		<< state_init(thetay_idx) << std::endl
		<< "Initial_yaw" << ","
		<< state_init(thetaz_idx) << std::endl
		<< "Prev_timestamp" << ","
		<< prev_timestamp << std::endl;
}

// Calculates the Initialization vector for Kalman Filter state and the values for gyro offset
Eigen::VectorXd set_init_val(CURL* curl, CURLcode res, std::string& read_buffer, boost::posix_time::ptime& request_time, boost::posix_time::ptime& response_time, const boost::posix_time::ptime& prev_timestamp, Sensors& garmin_imu, std::ofstream& log_file)
{
	Eigen::VectorXd acc_init(vec_size);
	Eigen::VectorXd gyro_init(vec_size);
	Eigen::VectorXd state_init(vec_size);
	int i = init_val;
	double sum_roll = init_val;
	double sum_pitch = init_val;
	double sum_yaw = init_val;
	double sum_gx = init_val;
	double sum_gy = init_val;
	double sum_gz = init_val;
	// Read a few values to calculate the average values that can be used for initialization
	while(i < init_loop_num)
	{
		curl_request_response(curl, res, data_command, read_buffer, request_time, response_time);
		garmin_imu.parse(read_buffer, read_buffer.length());
		read_buffer.clear();
		acc_init(thetax_idx) = (garmin_imu.get_sensor(internal_accelx).get_data() - offset::accx_offset) * g_acc;
		acc_init(thetay_idx) = (garmin_imu.get_sensor(internal_accely).get_data() - offset::accy_offset) * g_acc;
		acc_init(thetaz_idx) = (garmin_imu.get_sensor(internal_accelz).get_data() - offset::accz_offset) * g_acc;
		gyro_init(thetax_idx) = (garmin_imu.get_sensor(internal_gyrox).get_data());
		gyro_init(thetay_idx) = (garmin_imu.get_sensor(internal_gyroy).get_data());
		gyro_init(thetaz_idx) = (garmin_imu.get_sensor(internal_gyroz).get_data());
		acc_euler(acc_init, state_init);
		sum_gx +=  gyro_init(thetax_idx);
		sum_gy +=  gyro_init(thetay_idx);
		sum_gz +=  gyro_init(thetaz_idx);
		sum_roll += state_init(thetax_idx);
		sum_pitch += state_init(thetay_idx);
		sum_yaw += state_init(thetaz_idx);
		i++;
	}
	// Set average as initial value for both state and gyro offset
	offset::gyrox_offset = sum_gx / init_loop_num;
	offset::gyroy_offset = sum_gy / init_loop_num;
	offset::gyroz_offset = sum_gz / init_loop_num;
	state_init << sum_roll / init_loop_num, sum_pitch / init_loop_num, sum_yaw / init_loop_num;
	// Log the initial values
	log_init(log_file, state_init, prev_timestamp);
	return state_init;
}

// Reads the gyro rates and accelerometer values. Converts accelerometer values to roll, pitch and yaw angles
void read_values(const Sensors& garmin_imu, Eigen::VectorXd& u_ctrl, Eigen::VectorXd& z_obs, Eigen::VectorXd& z_obs_reading)
{
	// Gyro rates in rad/ s
	u_ctrl(thetax_idx) = (garmin_imu.get_sensor(internal_gyrox).get_data() - offset::gyrox_offset) * deg_to_rad;
	u_ctrl(thetay_idx) = (garmin_imu.get_sensor(internal_gyroy).get_data() - offset::gyroy_offset) * deg_to_rad;
	u_ctrl(thetaz_idx) = (garmin_imu.get_sensor(internal_gyroz).get_data() - offset::gyroz_offset) * deg_to_rad;

	// Accelerometer values in m/ s^2
	z_obs_reading(thetax_idx) = (garmin_imu.get_sensor(internal_accelx).get_data() - offset::accx_offset) * g_acc;
	z_obs_reading(thetay_idx) = (garmin_imu.get_sensor(internal_accely).get_data() - offset::accy_offset) * g_acc;
	z_obs_reading(thetaz_idx) = (garmin_imu.get_sensor(internal_accelz).get_data() - offset::accz_offset) * g_acc;

	// Conversion of accelerometer readings to roll, pitch and yaw
	acc_euler(z_obs_reading, z_obs, init_val);
}

// Set the yaw in actual observation to follow predicted observation. See the comments in the main prior to the call for below function.
void set_yaw(const Kalman_Filter& garmin_imu_kf, Eigen::VectorXd& predicted_obs, Eigen::VectorXd& z_obs)
{
	predicted_obs = garmin_imu_kf.get_predicted_obs();
	z_obs(thetaz_idx) = predicted_obs(thetaz_idx);
}

// Log all required values from a timestep
void log_timestep(const boost::posix_time::ptime& start_timestamp, const boost::posix_time::ptime& curr_timestamp, const Kalman_Filter& garmin_imu_kf, std::ofstream& log_file, int& log_num, const Eigen::VectorXd z_obs_reading, const Eigen::VectorXd predicted_obs)
{
	// Set header along with the first timestep log
	if (log_num == init_val)
	{
		log_file << "Log #" << ","
				 << "Timestamp (yyyy-mmm-dd hh:mm:ss.uuuuuu)" << ","
				 << "Time_elapsed (s)" << ","
				 << "Roll (rad)" << ","
				 << "Pitch (rad)" << ","
				 << "Yaw (rad)" << ","
				 << "Gx (rad/s)" << ","
				 << "Gy (rad/s)" << ","
				 << "Gz (rad/s)" << ","
				 << "Ax (m/s/s)" << ","
				 << "Ay (m/s/s)" << ","
				 << "Az (m/s/s)" << ","
				 << "Obs_roll (rad)" << ","
				 << "Obs_pitch (rad)" << ","
				 << "Obs_yaw (rad)" << ","
				 << "Pred_obs_roll (rad)" << ","
				 << "Pred_obs_pitch (rad)" << ","
				 << "Pred_obs_yaw (rad)" << std::endl;
	}
	Eigen::VectorXd curr_state = garmin_imu_kf.get_state();
	Eigen::VectorXd u_ctrl = garmin_imu_kf.get_control();
	Eigen::VectorXd z_obs = garmin_imu_kf.get_measurement();
	// Elapsed time is calculated for ease of result analysis
	double elapsed_time = ((curr_timestamp - start_timestamp).ticks()) * micro;
	std::ostringstream os;
	os << std::fixed << std::setprecision(decimal_points) << elapsed_time;
	std::string elapsed_time_str = os.str();

	// Log the values of timestamp, elapsed time, state, control input, observation reading, actual observation and predicted observation.
	log_file << log_num << ","
			 << curr_timestamp << ","
			 << elapsed_time_str << ","
			 << curr_state(thetax_idx) << ","
			 << curr_state(thetay_idx) << ","
			 << curr_state(thetaz_idx) << ","
			 << u_ctrl(thetax_idx) << ","
			 << u_ctrl(thetay_idx) << ","
			 << u_ctrl(thetaz_idx) << ","
			 << z_obs_reading(thetax_idx) << ","
			 << z_obs_reading(thetay_idx) << ","
			 << z_obs_reading(thetaz_idx) << ","
			 << z_obs(thetax_idx) << ","
			 << z_obs(thetay_idx) << ","
			 << z_obs(thetaz_idx) << ","
			 << predicted_obs(thetax_idx) << ","
			 << predicted_obs(thetay_idx) << ","
			 << predicted_obs(thetaz_idx) << std::endl;
	// Increment log record number
	log_num++;
}

// Print state at every timestep, onto the terminal
void print_timestep(const boost::posix_time::ptime& curr_timestamp, const Kalman_Filter& garmin_imu_kf)
{
	std::cout << "Garmin_IMU State -> Current Timestamp: " << curr_timestamp << std::endl;
	std::cout << garmin_imu_kf.get_state().format(CommaInitFmt) << std::endl;
}

// The main
int main(void)
{
	// cURL object
	CURL* curl;
	// cURL result object
	CURLcode res;
	// String to store cURL response
	std::string read_buffer;
	// Initialization of cURL object
	curl = curl_easy_init();
	// The request and response time of cURL HTTP post
	boost::posix_time::ptime request_time;
	boost::posix_time::ptime response_time;
	// The Eigen library vector to store the state of Kalman Filter in rad
	Eigen::VectorXd state{vec_size};
	// To store gyro angle rates in rad/s
	Eigen::VectorXd u_ctrl{vec_size};
	// To store actual observation in rad
	Eigen::VectorXd z_obs{vec_size};
	// To store predicted observation in rad
	Eigen::VectorXd predicted_obs{vec_size};
	// To store accelerometer reading in m/s^2
	Eigen::VectorXd z_obs_reading{vec_size};
	// The Sensors object that handles data communication with the camera and its storage
	Sensors garmin_imu;
	// The Extended Kalman Filter object
	Kalman_Filter garmin_imu_kf;
	// The recording of previous, current and start timestamps
	boost::posix_time::ptime prev_timestamp = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime curr_timestamp = prev_timestamp;
	boost::posix_time::ptime start_timestamp = prev_timestamp;
	// The time required between corresponding points in two different loops
	double loop_time = init_val;
	// The output file stream object for logging purposes
	std::ofstream log_file{fname_beg + boost::posix_time::to_simple_string(curr_timestamp) + file_format};
	// To count the log records
	int log_num = init_val;
	// Initialization of state and gyro offset
	state = set_init_val(curl, res, read_buffer, request_time, response_time, prev_timestamp, garmin_imu, log_file);
	garmin_imu_kf.set_state(state);
	// Print the state at initial timestep
	print_timestep(curr_timestamp, garmin_imu_kf);
	// Loop for Kalman Filter estimation
	while(true)
	{
		if(curl)
		{
			// Send cURL request to the camera and recieve a response
			curl_request_response(curl, res, data_command, read_buffer, request_time, response_time);
			// Parse the response from the camera to Sensors object
			garmin_imu.parse(read_buffer, read_buffer.length());
			// Clear the response buffer for next loop
			read_buffer.clear();
			// Time for timestep is the average of request and response time
			curr_timestamp = timestamp_midpoint(request_time, response_time);
			// Loop time in seconds obtained when ticks are multiplied with micro
			loop_time = ((curr_timestamp - prev_timestamp).ticks()) * micro;
			// Read the gyro rates and accelerometer readings. Set observation euler angles based on accelerometer readings.
			read_values(garmin_imu, u_ctrl, z_obs, z_obs_reading);
			// Pass the gyro rates to Kalman Filter as control input
			garmin_imu_kf.set_control(u_ctrl);
			// Prediction step of Kalman Filter
			garmin_imu_kf.prediction(loop_time);
			// Calculation of Kalman Gain factor
			garmin_imu_kf.kalman_gain_calc();
			/* Set the yaw of actual observation to follow the predicted observation. This is done because the accelerometer will not be able to correct for yaw as change in yaw in the original frame will not be reflected in accelerometer reading. So the idea is to trust the gyro integration of yaw for short term, without any correction. This is done keeping in mind the future requirements of the project. The following step ensures that the gyro integration error does not affect the calculation of innovation, and thus the estimates of roll and pitch. */
			set_yaw(garmin_imu_kf, predicted_obs, z_obs);
			// Pass the actual observation to Kalman Filter
			garmin_imu_kf.set_measurement(z_obs);
			// Correction step of Kalman Filter
			garmin_imu_kf.correction();
			// Print the state at current timestep
			print_timestep(curr_timestamp, garmin_imu_kf);
			// Log all required values from current timestep, for result analysis
			log_timestep(start_timestamp, curr_timestamp, garmin_imu_kf, log_file, log_num, z_obs_reading, predicted_obs);
			// Set current timestamp as previous timestamp for the next loop
			prev_timestamp = curr_timestamp;
		}

	}
	// Clean up
	log_file.close();
	curl_easy_cleanup(curl);
	return 0;
}

// The end
