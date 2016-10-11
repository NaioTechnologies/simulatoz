#ifndef CORE_HPP
#define CORE_HPP

#include <iostream>
#include <thread>
#include <mutex>

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/NavSatFix.h"

#include <tf/transform_listener.h>

#include "../include/oz440_api/Naio01Codec.hpp"
#include "../include/oz440_socket/ServerSocket.h"

class Core
{
public:
    // Constructeur/destructeur
    Core( int argc, char **argv );
    ~Core();

    void run( );

    // callback functions
    void send_lidar_packet_callback( const sensor_msgs::LaserScan::ConstPtr& lidar_msg );
    void send_camera_packet_callback(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right);
    void send_imu_packet_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void send_gps_packet_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg );
//    void send_odo_packet_callback(const gazebo_msgs::LinkStates::ConstPtr& odom_msg);
    void send_odo_packet();
    double getPitch( std::string wheel);
    bool odo_wheel( uint8_t & wheel, double& pitch, double& pitch_last_tic, int& forward_backward);

    // wait for core end.
    void join_client_read_thread();

private:
    // thread function
    void client_read_thread_function( );

private:
    std::shared_ptr<tf::TransformListener> listener_ptr_;

    bool stop_client_read_thread_asked_;
    bool client_read_thread_started_;
    std::thread client_read_thread_;
    std::thread send_odo_thread_;

//    std::shared_ptr< ServerSocket > server_socket_ptr_;
//    std::shared_ptr< ServerSocket > accepted_socket_ptr_;

    ServerSocket *server_socket_ptr_;
    ServerSocket *accepted_socket_ptr_;

    std::mutex packet_to_send_list_access_;
    std::vector< BaseNaio01PacketPtr > packet_to_send_list_;

    // ROS PART
    ros::Publisher velocity_pub_;
};

#endif //CORE_HPP
