#ifndef CORE_HPP
#define CORE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <array>
#include <atomic>

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/NavSatFix.h"

#include <tf/transform_listener.h>

#include "ApiStereoCameraPacket.hpp"

#include "Camera.h"
#include "Can.h"
#include "Serial.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>

class RosLidar;

class Core
{
public:
	Core( int argc, char** argv );

	~Core();

	void run();

private:

	void callback_top_camera( const sensor_msgs::Image::ConstPtr& image );

	bool setup_video_folder();

private:
	bool terminate_;

	std::string log_folder_;

	bool use_camera_;
	std::shared_ptr< Camera > camera_ptr_;
	int camera_port_;

	std::unique_ptr< RosLidar > lidar_;

	bool use_can_;
	std::shared_ptr< Can > can_ptr_;
	int can_port_;

	std::shared_ptr< Serial > serial_ptr_;
	int serial_port_;

	// Video_recorder
	std::string video_folder_;
	cv::VideoWriter output_video_;
};

#endif //CORE_HPP
