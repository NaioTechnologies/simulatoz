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

#include "Serial.h"
#include "Camera.h"
#include "Can.h"
#include "Lidar.h"

#include "VideoLog.h"
#include "Log.h"
#include "Metric.hpp"

class Core
{
public:
	Core( int argc, char** argv );
    void run();

private:
    // callback functions
    void callback_actuator_position( const sensor_msgs::JointState::ConstPtr& joint_states_msg );
    void callback_imu(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg );

    // Odometry part
    void odometry_thread();
    double getPitch( std::string wheel);
    bool odo_wheel( uint8_t & wheel, double& pitch, double& pitch_last_tic, int& forward_backward);

private:

    bool terminate_ ;

    std::shared_ptr<tf::TransformListener> listener_ptr_;

    bool use_camera_;
    std::shared_ptr<Camera> camera_ptr_;
    int camera_port_;

    bool use_lidar_;
    std::shared_ptr<Lidar> lidar_ptr_;
    int lidar_port_;

    bool use_can_;
    std::shared_ptr<Can> can_ptr_;
    int can_port_;

    std::shared_ptr<Serial> serial_ptr_;
    uint serial_port_

    // ROS PART
    ros::Publisher actuator_pub_;

    std::shared_ptr<VideoLog> video_log_ptr_;
    std::string video_log_folder_;

    std::shared_ptr<Log> log_ptr_;
    std::string log_folder_;
    std::shared_ptr<Metric> metric_ptr_;
};

#endif //CORE_HPP
