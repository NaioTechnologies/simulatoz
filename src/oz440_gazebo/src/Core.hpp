#ifndef CORE_HPP
#define CORE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <array>
#include <atomic>

#include "ros/ros.h"

#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/NavSatFix.h"

#include "Serial.h"
#include "Camera.h"
#include "Lidar.h"
#include "Can.h"

#include "VideoLog.h"
#include "Log.h"
#include "Metric.hpp"
#include "Odometry.h"

class Core
{
public:

    Core( int argc, char **argv );
    ~Core();

    void run();

private:

    bool use_camera_;
    std::shared_ptr<Camera> camera_ptr_;
    int camera_port_;

    bool use_lidar_;
    std::shared_ptr<Lidar> lidar_ptr_;
    int lidar_port_;

    bool use_can_;
    std::shared_ptr<Can> can_ptr_;
    std::shared_ptr<Odometry> odometry_ptr_;
    int can_port_;

    std::shared_ptr<Serial> serial_ptr_;
    uint serial_port_;

    float actuator_position_;

    std::shared_ptr<VideoLog> video_log_ptr_;
    std::string video_log_folder_;

    std::shared_ptr<Log> log_ptr_;
    std::string log_folder_;
    std::shared_ptr<Metric> metric_ptr_;




};

#endif //CORE_HPP
