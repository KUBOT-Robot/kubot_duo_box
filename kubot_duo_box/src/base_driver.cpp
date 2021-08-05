#include "base_driver.h"
#include "data_holder.h"
#include "simple_dataframe_master.h"
#include "serial_transport2.h"

#include <std_msgs/Float32MultiArray.h>
#include <boost/assign/list_of.hpp>
#include <string.h> 

BaseDriver* BaseDriver::instance = NULL;

BaseDriver::BaseDriver() : pn("~"), bdg(pn)
{
	//init config
	bdg.init(&Data_holder::get()->parameter);

	trans = boost::make_shared<Serial_transport2>(bdg.port, bdg.baudrate);

	frame = boost::make_shared<Simple_dataframe>(trans.get());

	ROS_INFO("[KUBOT]due box driver startup...");
	if (trans->init())
	{
		ROS_INFO("[KUBOT]connected to due box board");
	}
	else
	{
		ROS_ERROR("[KUBOT]oops!!! can't connect to due box board, please check the usb connection or baudrate!");
		return;
	}

	ros::Duration(3).sleep(); //wait for device
	ROS_INFO("[KUBOT]wait for device...");

	frame->init();

	for (int i = 0;i < 3;i++) {
		if (frame->interact(ID_GET_VERSION))
			break;
		ros::Duration(2).sleep(); //wait for device
	}

	ROS_INFO("[KUBOT]robot version:%s build time:%s",
		Data_holder::get()->firmware_info.version,
		Data_holder::get()->firmware_info.time);

	init_duo_box();

	init_led_control();

	init_servo_control();
}

BaseDriver::~BaseDriver()
{
	if (instance != NULL)
		delete instance;
}

void BaseDriver::init_duo_box()
{

}

void BaseDriver::init_led_control()
{
	ROS_INFO_STREAM("[KUBOT]advertise led topic on [" << bdg.led_status_topic << "]");
	led_control_pub = nh.advertise<bdg.led_status_topic>(bdg.led_status_topic, 50);
	led_status_msgs.header.frame_id = "led_status";

	ROS_INFO_STREAM("[KUBOT]subscribe led topic on [" << bdg.led_control_topic << "]");
	led_control_sub = nh.subscribe(bdg.led_control_topic, 50, &BaseDriver::update_led_status, this);
	led_control_msgs.header.frame_id = "led_control";
}

void BaseDriver::init_servo_control()
{
	ROS_INFO_STREAM("[KUBOT]advertise servo topic on [" << bdg.servo_status_topic << "]");
	servo_control_pub = nh.advertise<bdg.servo_status_topic>(bdg.servo_status_topic, 50);
	servo_status_msgs.header.frame_id = "servo_status";

	ROS_INFO_STREAM("[KUBOT]subscribe servo topic on [" << bdg.servo_control_topic << "]");
	servo_control_sub = nh.subscribe(bdg.servo_control_topic, 50, &BaseDriver::update_servo_status, this);
	servo_control_msgs.header.frame_id = "servo_control";
}

void BaseDriver::work_loop()
{
	ros::Rate loop(bdg.freq);

	while (ros::ok()) {

		get_led_status();

		get_servo_status();

		loop.sleep();

		ros::spinOnce();
	}
}

int UPDATE_LED_STATUS_INTERVAL = 1;
void BaseDriver::get_led_status()
{
	static int last_millis = 0;
	if (ros::Time::now().toSec() - last_millis > UPDATE_LED_STATUS_INTERVAL) {
		frame->interact(ID_GET_LED_STATUS);
		led_status_msgs.header.stamp = ros::Time::now();

		led_status_msgs.ledNum = Data_holder::get()->led_status.ledNum;
		led_status_msgs.led_brightness = Data_holder::get()->led_status.led_brightness;
		led_status_msgs.led_speed = Data_holder::get()->led_status.led_speed;
		led_status_msgs.led_color_r = Data_holder::get()->led_status.led_color_r;
		led_status_msgs.led_color_g = Data_holder::get()->led_status.led_color_g;
		led_status_msgs.led_color_b = Data_holder::get()->led_status.led_color_b;
		led_status_msgs.led_mode = Data_holder::get()->led_status.led_mode;

		led_status_pub.publish(led_status_msgs);

		last_millis = ros::Time::now().toSec();
	}
}

int UPDATE_SERVO_STATUS_INTERVAL = 1;
void BaseDriver::get_servo_status()
{
	static int last_millis = 0;
	if (ros::Time::now().toSec() - last_millis > UPDATE_SERVO_STATUS_INTERVAL) {
		frame->interact(ID_GET_SERVO_STATUS);
		servo_status_msgs.header.stamp = ros::Time::now();

		servo_status_msgs.servoNum = Data_holder::get()->servo_status.servoNum;
		servo_status_msgs.servo_angle = Data_holder::get()->servo_status.servo_angle;

		servo_status_pub.publish(servo_status_msgs);

		last_millis = ros::Time::now().toSec();
	}
}