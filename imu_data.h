// Header for the classes and functions responsible for data access from the GARMIN VIRB IMU sensors. The implementation file is imu_data.cpp.
#ifndef IMU_DATA_H
#define IMU_DATA_H
#include <unordered_map>

// The class to keep the data of individual sensors
class Sensors_Object
{
	private:
		std::string name;
		std::string type;
		int has_data;
		std::string units;
		std::string data_type;
		double data;
	public:
		Sensors_Object();
		Sensors_Object(std::string, std::string, int, std::string, std::string, double);
		Sensors_Object(const Sensors_Object&);
		void print_properties() const;
		double get_data() const;
};

// The class to keep the map of sensors. The key used is the name of the sensor; for example: "InetrnalGyroX".
class Sensors
{
	private:
		std::unordered_map<std::string, Sensors_Object> sensors;
	public:
		void parse(const std::string&, size_t);
		Sensors_Object get_sensor(const std::string&) const;
};

// The address to connect with the GARMIN VIRB camera.
constexpr auto curl_url = "http://192.168.0.1/virb"; 
// Parameter to indicate regular HTTP POST
constexpr long post_param = 1;
// Starting index for sensor list
constexpr int sensor_list_sindex = 0;
// Default values for sensor fields
const std::string str_default = "Invalid";
const std::string double_default = "0.00";
const std::string int_default = "0";
// String literals to access sensor fields
const std::string jpl_sensors = "sensors";
const std::string jpl_name = "name";
const std::string jpl_type = "type";
const std::string jpl_has_data = "has_data";
const std::string jpl_units = "units";
const std::string jpl_data_type = "data_type";
const std::string jpl_data = "data";

// The callback function used by cURL to obtain responses from GARMIN VIRB server
static size_t WriteCallback(void* , size_t, size_t, void*);
// The function uses cURL to communicate with the GARMIN VIRB server
void curl_request_response(CURL*, CURLcode, const std::string&, std::string&, boost::posix_time::ptime&, boost::posix_time::ptime&);

#endif
