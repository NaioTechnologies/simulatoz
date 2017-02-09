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
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/NavSatFix.h"

#include <tf/transform_listener.h>

#include "oz440_socket/ServerSocket.h"
#include "oz440_api/ApiStereoCameraPacket.hpp"

#include "Camera.h"
#include "Lidar.h"
#include "Can.h"
#include "Serial.h"
#include "ThreadsafeQueue.hpp"

#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include "oz440_api/CLFileSystem.h"


class Core
{
public:
    Core( int argc, char **argv );
    ~Core();

    void run();

private:

    // thread function
    void read_thread_function( );

    // callback functions
    void callback_lidar( const sensor_msgs::LaserScan::ConstPtr& lidar_msg );
    void callback_actuator_position( const sensor_msgs::JointState::ConstPtr& joint_states_msg );
    void callback_camera(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right);
    void callback_imu(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg );
    void callback_top_camera(const sensor_msgs::Image::ConstPtr& image );

    // Odometry part
    void odometry_thread();
    double getPitch( std::string wheel);
    bool odo_wheel( uint8_t & wheel, double& pitch, double& pitch_last_tic, int& forward_backward);
    bool setup_video_folder();

private:

    bool terminate_ ;

    std::string log_folder_;

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
    int serial_port_;

    std::vector< BaseNaio01PacketPtr > received_packet_list_;

    bool read_thread_started_;
    std::thread read_thread_;

    bool odometry_thread_started_;
    std::thread odometry_thread_;;

    float actuator_position_;

    // ROS PART
    ros::Publisher velocity_pub_;
    ros::Publisher actuator_pub_;

    // Video_recorder
    std::string video_folder_;
    cv::VideoWriter output_video_;
};

#endif //CORE_HPP
