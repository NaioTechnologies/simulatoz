cmake_minimum_required( VERSION 2.8.3 )
project( oz440_msgs )

find_package( catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs )

add_message_files( FILES
				   Odometry.msg
				   Screen.msg
				   Motors.msg )

add_service_files( FILES )

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS
	std_msgs )
