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
	bdg.init();

	trans = boost::make_shared<Serial_transport2>(bdg.port, bdg.baudrate);

	frame = boost::make_shared<Simple_dataframe>(trans.get());

	ROS_INFO("[KUBOT]duo box driver startup...");
	if (trans->init())
	{
		ROS_INFO("[KUBOT]connected to duo box board");
	}
	else
	{
		ROS_ERROR("[KUBOT]oops!!! can't connect to duo box board, please check the usb connection or baudrate!");
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
	ROS_INFO_STREAM("[KUBOT]advertise led topic on [" << bdg.led_status_topic << "]");
	led_status_pub = nh.advertise<kubot_duo_msgs::RawLedp>("led_status", 50);
	led_status_msgs.header.frame_id = "led_status";

	ROS_INFO_STREAM("[KUBOT]advertise servo topic on [" << bdg.servo_status_topic << "]");
	servo_status_pub = nh.advertise<kubot_duo_msgs::RawServop>("servo_status", 50);
	servo_status_msgs.header.frame_id = "servo_status";
}

void BaseDriver::init_led_control()
{

	ROS_INFO_STREAM("[KUBOT]subscribe led topic on [" << bdg.led_control_topic << "]");
	led_control_sub = nh.subscribe(bdg.led_control_topic, 50, &BaseDriver::callback_led_status, this);
	led_control_msgs.header.frame_id = "led_control";

	need_update_led = false;
}

void BaseDriver::init_servo_control()
{

	ROS_INFO_STREAM("[KUBOT]subscribe servo topic on [" << bdg.servo_control_topic << "]");
	servo_control_sub = nh.subscribe(bdg.servo_control_topic, 50, &BaseDriver::callback_servo_status, this);
	servo_control_msgs.header.frame_id = "servo_control";

	need_update_servo = false;
}

void BaseDriver::callback_led_status(const kubot_duo_msgs::RawLeds& led_control_msgs) {

	Data_holder::get()->led_status.ledNum = led_control_msgs.ledNum;
	Data_holder::get()->led_status.led_brightness = led_control_msgs.led_brightness;
	Data_holder::get()->led_status.led_speed = led_control_msgs.led_speed;
	Data_holder::get()->led_status.led_color_r = led_control_msgs.led_color_r;
	Data_holder::get()->led_status.led_color_g = led_control_msgs.led_color_g;
	Data_holder::get()->led_status.led_color_b = led_control_msgs.led_color_b;
	Data_holder::get()->led_status.led_mode = led_control_msgs.led_mode;

	need_update_led = true;
}

void BaseDriver::callback_servo_status(const kubot_duo_msgs::RawServos& servo_control_msgs) {

	Data_holder::get()->servo_status.servoNum = servo_control_msgs.servoNum;
	Data_holder::get()->servo_status.servo_angle = servo_control_msgs.servo_angle;

	need_update_servo = true;
}

void BaseDriver::work_loop()
{
	ros::Rate loop(bdg.freq);

	while (ros::ok()) {

		get_led_status();

		get_servo_status();

		update_led_status();

		update_servo_status();

		loop.sleep();

		ros::spinOnce();
	}
}

int UPDATE_LED_STATUS_INTERVAL = 2;
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

int UPDATE_SERVO_STATUS_INTERVAL = 2;
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

void BaseDriver::update_led_status() {

	if (need_update_led) {
		ROS_INFO_STREAM("update led");
		need_update_led = !(frame->interact(ID_SET_LED_STATUS));
	}
}

void BaseDriver::update_servo_status() {

	if (need_update_servo) {
		ROS_INFO_STREAM("update servo");
		need_update_servo = !(frame->interact(ID_SET_SERVO_STATUS));
	}
}