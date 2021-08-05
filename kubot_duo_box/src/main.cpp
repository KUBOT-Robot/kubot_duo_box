#include <ros/ros.h>

#include "base_driver.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "kubot_due_box");

	BaseDriver::Instance()->work_loop();

	ros::spin();

	return 0;
}