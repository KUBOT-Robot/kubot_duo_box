#include <ros/ros.h>
#include "kubot_duo_box/base_driver.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "kubot_duo_box");

	BaseDriver::Instance()->work_loop();

	ros::spin();

	return 0;
}