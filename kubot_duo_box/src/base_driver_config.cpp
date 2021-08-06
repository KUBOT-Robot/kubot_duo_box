#include "base_driver_config.h"
#include "data_holder.h"

#define PI 3.1415926f

DueBoxDriverConfig::DueBoxDriverConfig(ros::NodeHandle &p) : pn(p)
{

}

DueBoxDriverConfig::~DueBoxDriverConfig()
{
}

void DueBoxDriverConfig::init()
{
	// Comm param
	pn.param<std::string>("port", port, "/dev/ttyACM0");
	pn.param<int32_t>("baudrate", baudrate, 115200);

	ROS_INFO("[KUBOT]port:%s baudrate:%d", port.c_str(), baudrate);

	pn.param<bool>("led_control_enable", led_control_enable, true);
	ROS_INFO("[KUBOT]led_control_enable:%d", led_control_enable);
	pn.param<bool>("servo_control_enable", servo_control_enable, true);
	ROS_INFO("[KUBOT]servo_control_enable:%d", servo_control_enable);

	// Topic name param
	pn.param<std::string>("led_status_topic", led_status_topic, "led_status");
	pn.param<std::string>("led_control_topic", led_control_topic, "led_control");
	pn.param<std::string>("servo_status_topic", servo_status_topic, "servo_status");
	pn.param<std::string>("servo_control_topic", servo_control_topic, "servo_status");

	pn.param<int32_t>("freq", freq, 1000);
}