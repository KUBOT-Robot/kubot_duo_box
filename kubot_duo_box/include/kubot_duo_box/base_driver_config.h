#ifndef KUBOT_BASE_DRIVER_CONFIG_
#define KUBOT_BASE_DRIVER_CONFIG_

#include <ros/ros.h>

#define USE_DYNAMIC_RECONFIG

#ifdef USE_DYNAMIC_RECONFIG
#include <dynamic_reconfigure/server.h>
#include "kubot_duo_box/kubot_driverConfig.h"
#endif

class Controller_robot_parameter;

class DueBoxDriverConfig
{
public:
	DueBoxDriverConfig(ros::NodeHandle& p);
	~DueBoxDriverConfig();
	void init(Controller_robot_parameter* r);
private:
	ros::NodeHandle& pn;
	ros::ServiceClient client;

	// 定義機器人參數	
public:
	void SetRobotParameters();
	Controller_robot_parameter* rp;

#ifdef USE_DYNAMIC_RECONFIG
	void dynamic_callback(kubot_duo_box::kubot_driverConfig& config, uint32_t level);
	bool get_param_update_flag();

private:
	dynamic_reconfigure::Server<kubot_duo_box::kubot_driverConfig> server;
	dynamic_reconfigure::Server<kubot_duo_box::kubot_driverConfig>::CallbackType f;
#endif
private:
	bool set_flag;
#ifdef USE_DYNAMIC_RECONFIG
	bool param_update_flag;
#endif

	// 定義通訊串口
public:
	std::string port;
	int32_t baudrate;

	// 定義啟動功能開關
public:
	bool led_control_enable;
	bool servo_control_enable;

	// 定義topic
public:
	std::string led_status_topic;
	std::string led_control_topic;
	std::string servo_status_topic;
	std::string servo_control_topic;

	int32_t freq;
};

#endif /* KUBOT_BASE_DRIVER_CONFIG_H_ */