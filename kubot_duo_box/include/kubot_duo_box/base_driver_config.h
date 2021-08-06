#ifndef KUBOT_BASE_DRIVER_CONFIG_
#define KUBOT_BASE_DRIVER_CONFIG_

#include <ros/ros.h>

class DueBoxDriverConfig
{
public:
	DueBoxDriverConfig(ros::NodeHandle& p);
	~DueBoxDriverConfig();
	void init();

public:
// 定義通訊串口
	std::string port;
	int32_t baudrate;

// 定義啟動功能開關
	bool led_control_enable;
	bool servo_control_enable;

// 定義topic name
	std::string led_status_topic;
	std::string led_control_topic;
	std::string servo_status_topic;
	std::string servo_control_topic;

	int32_t freq;

private:
	ros::NodeHandle& pn;
	ros::ServiceClient client;
};

#endif /* KUBOT_BASE_DRIVER_CONFIG_H_ */