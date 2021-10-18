#include "kubot_duo_box/data_holder.h"

#if _DATAHOLDER_IN_DEVICES
#include "board.h"

void Data_holder::load_parameter()
{
	Board::get()->get_config((unsigned char*)&parameter, sizeof(parameter));
}

void Data_holder::save_parameter()
{
	Board::get()->set_config((unsigned char*)&parameter, sizeof(parameter));
}
#else
#include <ros/ros.h>
#endif

void Data_holder::dump_params(struct Controller_robot_parameter* params)
{
#if _DATAHOLDER_IN_DEVICES
#else

	ROS_INFO("[KUBOT]RobotParameters:\n \
                        \t\t Led Pixel : %d \n \
                        \t\t Servo Motor Max Angle :  %d \n \
                        \t\t Servo Motor Min Angle :  %d \n \
                         "
		, params->led_pixel
		, params->servo_max
		, params->servo_min
	);
#endif
}
