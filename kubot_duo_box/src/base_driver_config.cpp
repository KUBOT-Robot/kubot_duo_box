#include "base_driver_config.h"
#include "data_holder.h"

#define PI 3.1415926f

DueBoxDriverConfig::DueBoxDriverConfig(ros::NodeHandle& p) : pn(p)
{
#ifdef USE_DYNAMIC_RECONFIG
	param_update_flag = false;
#endif
	set_flag = true;
}

DueBoxDriverConfig::~DueBoxDriverConfig()
{
}

void DueBoxDriverConfig::init(Controller_robot_parameter* r)
{
	rp = r;

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
	pn.param<std::string>("servo_control_topic", servo_control_topic, "servo_control");

	pn.param<int32_t>("freq", freq, 1000);
}

void DueBoxDriverConfig::SetRobotParameters()
{
#ifdef USE_DYNAMIC_RECONFIG
	static bool flag = true;
	if (flag)
	{
		flag = false;
		f = boost::bind(&DueBoxDriverConfig::dynamic_callback, this, _1, _2);
		server.setCallback(f);
	}
#endif
}

#ifdef USE_DYNAMIC_RECONFIG
void DueBoxDriverConfig::dynamic_callback(kubot_duo_box::kubot_driverConfig &config, uint32_t level)
{
	if (set_flag)
	{
		set_flag = false;
		config.led_pixel = rp->led_pixel;
		config.servo_max = rp->servo_max;
		config.servo_min = rp->servo_min;
		return;
	}

	rp->led_pixel = config.led_pixel;
	rp->servo_max = config.servo_max;
	rp->servo_min = config.servo_min;

	Data_holder::dump_params(rp);

	param_update_flag = true;
}

bool DueBoxDriverConfig::get_param_update_flag()
{
	bool tmp = param_update_flag;
	param_update_flag = false;

	return tmp;
}

#endif
