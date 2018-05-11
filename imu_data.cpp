// The header files
#include <iostream>
#include <string>
#include <curl/curl.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <jsoncpp/json/json.h>
#include <unordered_map>
#include "imu_data.h"

// The default constructor
Sensors_Object::Sensors_Object()
{

}

// Constructor 2
Sensors_Object::Sensors_Object(std::string name_p, std::string type_p, int has_data_p, std::string units_p, std::string data_type_p, double data_p)
{
	name = name_p;
	type = type_p;
	has_data = has_data_p;
	units = units_p;
	data_type = data_type_p;
	data = data_p;
}

// Constructor 3
Sensors_Object::Sensors_Object(const Sensors_Object& s_obj)
{
	name = s_obj.name;
	type = s_obj.type;
	has_data = s_obj.has_data;
	units = s_obj.units;
	data_type = s_obj.data_type;
	data = s_obj.data;	
}

// To print the object properties/ data fields
void Sensors_Object::print_properties() const
{
	std::cout << "Printing from Sensors_Object " << std::endl;
	std::cout << "Name: " << name << std::endl;
	std::cout << "Type: " << type << std::endl;
	std::cout << "Has_Data: " << has_data << std::endl;
	std::cout << "Units: " << units << std::endl;
	std::cout << "Data_Type: " << data_type << std::endl;
	std::cout << "Data: " << data << std::endl;
}

// Return the data field of the object
double Sensors_Object::get_data() const
{
	return data;
}

// Parse the data from JSON string to the individual sensor objects.
void Sensors::parse(const std::string& json_string, size_t sensors_size)
{
	Json::Value json_data;
	Json::CharReaderBuilder json_reader;
	std::string errs;
	std::stringstream json_ss;
	json_ss << json_string;

	std::string name;
	std::string type;
	int has_data;
	std::string units;
	std::string data_type;
	double data; 
	if (Json::parseFromStream(json_reader, json_ss, &json_data, &errs))
    	{
		std::cout << "Json Parsing Successful" << std::endl;
		const Json::Value sensor_list = json_data[jpl_sensors];
		// Each run of the loop will create a new indivdual sensor object as a Sensors_Object instance. Their map will be maintained by an instance of Sensors class.
		for (int i = sensor_list_sindex; i < sensor_list.size(); ++i)
		{
			name = sensor_list[i].get(jpl_name, str_default).asString();
			type = sensor_list[i].get(jpl_type, str_default).asString();
			has_data = stoi(sensor_list[i].get(jpl_has_data, int_default).asString());
			units = sensor_list[i].get(jpl_units, str_default).asString();
			data_type = sensor_list[i].get(jpl_data_type, str_default).asString();
			data = stod(sensor_list[i].get(jpl_data, double_default).asString());
			sensors[name] = {name, type, has_data, units, data_type, data};
		}
	}
}

// Return an individual sensor object using its name as key to the map maintained by Sensors instance.
Sensors_Object Sensors::get_sensor(const std::string& name) const
{
	return sensors.find(name) -> second;
}

// The callback function used by cURL to obtain responses from GARMIN VIRB server
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
	((std::string*)userp)->append((char*)contents, size * nmemb);
	return size * nmemb;
}

// The function uses cURL to communicate with the GARMIN VIRB server
void curl_request_response(CURL* curl, CURLcode res, const std::string& data_command, std::string& read_buffer, boost::posix_time::ptime& request_time, boost::posix_time::ptime& response_time)
{
	// Different settings/ data for POST
	// The URL to connect 
	curl_easy_setopt(curl, CURLOPT_URL, curl_url);
	// The JSON command to be POSTed
	curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data_command.c_str());
	curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, data_command.length());
	// The HTTPS POST step
	curl_easy_setopt(curl, CURLOPT_POST, post_param);
	// The Callback function to write data when a repsonse is received
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
	// Write the result data to a string
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);
	// Request time of POST
	request_time = boost::posix_time::microsec_clock::local_time();
	// The actual POST that makes use of the above settings
	res = curl_easy_perform(curl);
	// Response time of POST
	response_time = boost::posix_time::microsec_clock::local_time();
}

