// Header for functions and facilities in kalman_filter_imu.cpp
#ifndef Kalman_Filter_Imu_H
#define Kalman_Filter_Imu_H

// The command to be posted via cURL, to read sensor data
const std::string data_command = "{\"command\":\"sensors\"}";
// General purpose
constexpr double micro = 1e-6; 
// To convert degree to radian
constexpr double deg_to_rad = M_PI / 180;
// To set the number of readings taken for initialization
constexpr int init_loop_num = 1000;
// General purpose
constexpr int decimal_points = 6;
// To pick current timestamp based on request and repsonse timestamp of the sensor
constexpr int tstamp_divisor = 2;
// To name the log file
const std::string fname_beg = "log_ornt_";
const std::string file_format = ".csv";
// To read sensor values
const std::string internal_accelx = "InternalAccelX";
const std::string internal_accely = "InternalAccelY";
const std::string internal_accelz = "InternalAccelZ";
const std::string internal_gyrox = "InternalGyroX";
const std::string internal_gyroy = "InternalGyroY";
const std::string internal_gyroz = "InternalGyroZ";

// Sensor Offset Values
namespace offset
{
	double accx_offset = -0.0070;
	double accy_offset = -0.0175;
	double accz_offset = -0.0864;
	double gyrox_offset = 2.8097;
	double gyroy_offset = 0.9402;
	double gyroz_offset = -5.2413;
}

// To print the Kalman Filter state vector in proper format
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "<< ", ";");
// Returns the midpoint of two timestamps 
boost::posix_time::ptime timestamp_midpoint(const boost::posix_time::ptime&, const boost::posix_time::ptime&);
// Performs conversion raw accelerometer readings to roll, pitch and yaw observations
void acc_euler(const Eigen::VectorXd, Eigen::VectorXd&, const double);
// Log initial offsets, noise values, state and timestamp
void log_init(std::ofstream&, const Eigen::VectorXd, const boost::posix_time::ptime&);
// Calculates the Initialization vector for Kalman Filter state and the values for gyro offset
Eigen::VectorXd set_init_val(CURL*, CURLcode, std::string&, boost::posix_time::ptime&, boost::posix_time::ptime&, const boost::posix_time::ptime&, Sensors&, std::ofstream&);
// Reads the gyro rates and accelerometer values. Converts accelerometer values to roll, pitch and yaw angles 
void read_values(const Sensors&, Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&);
// Set the yaw in actual observation to follow predicted observation. See the comments in the main prior to calling the below function.
void set_yaw(const Kalman_Filter&, Eigen::VectorXd&, Eigen::VectorXd&);
// Log all required values from a timestep
void print_timestep(const boost::posix_time::ptime&, const Kalman_Filter&);
// Print state at every timestep, onto the terminal
void log_timestep(const boost::posix_time::ptime&, const boost::posix_time::ptime&, const Kalman_Filter&, std::ofstream&, int&, const Eigen::VectorXd, const Eigen::VectorXd);
#endif 
