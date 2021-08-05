#ifndef KUBOT_BASE_DRIVER_H_
#define KUBOT_BASE_DRIVER_H_

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include "base_driver_config.h"

#include "kubot_due_bringup/transport.h"
#include "kubot_due_bringup/dataframe.h"

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <kubot_duo_msgs/RawLedP.h>
#include <kubot_duo_msgs/RawLedS.h>
#include <kubot_duo_msgs/RawServoP.h>
#include <kubot_duo_msgs/RawServoS.h>

class BaseDriver
{
private:
	BaseDriver();
	ros::NodeHandle nh;
	ros::NodeHandle pn;
	DueBoxDriverConfig bdg;
	static BaseDriver* instance;

public:
	static BaseDriver* Instance()
	{
		if (instance == NULL)
			instance = new BaseDriver();

		return instance;
	}
	~BaseDriver();

	void work_loop();

public:
	DueBoxDriverConfig& getDueBoxDriverConfig()
	{
		return bdg;
	}

	ros::NodeHandle* getNodeHandle()
	{
		return &nh;
	}

	ros::NodeHandle* getPrivateNodeHandle()
	{
		return &pn;
	}

// 初始化
private:
	void init_duo_box();
	void init_led_control();
	void init_servo_control();

// 獲取LED狀態
private:
	void get_led_status();
	kubog_due_msgs::RawLedP led_status_msgs;
	ros::Publisher led_status_pub;

// 更新LED狀態
private:
	void update_led_status();
	kubot_due_msgs::RawLedS led_control_msgs;
	ros::Subscriber led_control_sub;

// 獲取舵機狀態
private:
	void get_servo_status();
	kubot_due_msgs::RawServoP servo_status_msgs;
	ros::Publisher servo_status_pub;

// 更新舵機狀態
private:
	void update_led_status();
	kubot_due_msgs::RawServoS servo_control_msgs;
	ros::Subscriber servo_control_sub;

};

#endif /* KUBOT_BASE_DRIVER_H_ */