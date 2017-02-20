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
#include "VideoLog.h"
#include "Log.h"
#include "Metric.hpp"

class RosLidar;

class Core
{
public:
	Core( int argc, char** argv );

	~Core();

	void run();

private:

private:
	bool terminate_;

	bool use_camera_;
	std::shared_ptr< Camera > camera_ptr_;
	int camera_port_;

	std::unique_ptr< RosLidar > lidar_;

	bool use_can_;
	std::shared_ptr< Can > can_ptr_;
	int can_port_;

	std::shared_ptr< Serial > serial_ptr_;
	int serial_port_;

    std::shared_ptr<VideoLog> video_log_ptr_;
    std::string video_log_folder_;

    std::shared_ptr<Log> log_ptr_;
    std::string log_folder_;
    std::shared_ptr<Metric> metric_ptr_;

};

#endif //CORE_HPP
