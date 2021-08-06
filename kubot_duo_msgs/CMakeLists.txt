cmake_minimum_required(VERSION 2.8.3)
project(kubot_duo_msgs)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   geometry_msgs
)

add_message_files(
   FILES
	RawLedP.msg
   RawLedS.msg
   RawServoP.msg
   RawServoS.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)