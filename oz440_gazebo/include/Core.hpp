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
#include "Bridge.hpp"
#include "Camera.h"
#include "Lidar.h"
#include "ThreadsafeQueue.hpp"

class Core
{
public:
    // Constructeur/destructeur
    Core( int argc, char **argv );
    ~Core();

    void run( int argc, char **argv );

    // callback functions
    void callback_lidar( const sensor_msgs::LaserScan::ConstPtr& lidar_msg );
    void callback_actuator_position( const sensor_msgs::JointState::ConstPtr& joint_states_msg );
    void callback_camera(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right);
    void callback_imu(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg );

    void odometry_thread();
    double getPitch( std::string wheel);
    bool odo_wheel( uint8_t & wheel, double& pitch, double& pitch_last_tic, int& forward_backward);


private:
    // thread function
    void read_thread_function( );
    void image_thread_function( );

    void bridge_thread_function();

    void disconnected();

    void ozcore_image_thread_function( );

    void disconnection_ozcore_image();

private:

    bool terminate_ ;

    std::shared_ptr<tf::TransformListener> listener_ptr_;
    std::shared_ptr<Bridge> bridge_ptr_;

    bool use_camera_;
    std::shared_ptr<Camera> camera_ptr_;
    int camera_port_;

    bool use_lidar_;
    std::shared_ptr<Lidar> lidar_ptr_;
    int lidar_port_;

    bool use_can_;
//    std::shared_ptr<Bridge> bridge_ptr_;

    std::vector< BaseNaio01PacketPtr > received_packet_list_;

    bool read_thread_started_;
    std::thread read_thread_;

    std::thread send_odo_thread_;;

    bool bridge_thread_started_;
    std::thread bridge_thread_;
    bool graphics_on_;

    float actuator_position_;

    // ROS PART
    ros::Publisher velocity_pub_;
    ros::Publisher actuator_pub_;

};

#endif //CORE_HPP
