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
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/NavSatFix.h"

#include <tf/transform_listener.h>

#include "../include/oz440_api/Naio01Codec.hpp"
#include "../include/oz440_socket/ServerSocket.h"
#include "oz440_api/ApiStereoCameraPacket.hpp"

class Core
{
public:
    // Constructeur/destructeur
    Core( int argc, char **argv );
    ~Core();

    void run( int argc, char **argv );

    // callback functions
    void send_lidar_packet_callback( const sensor_msgs::LaserScan::ConstPtr& lidar_msg );
    void send_actuator_position_callback( const sensor_msgs::JointState::ConstPtr& joint_states_msg );
    void send_camera_packet_callback(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right);
    void send_imu_packet_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void send_gps_packet_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg );

    void send_odo_packet();
    double getPitch( std::string wheel);
    bool odo_wheel( uint8_t & wheel, double& pitch, double& pitch_last_tic, int& forward_backward);
    void distort_image(uint8_t image_ptr[ 721920 ], float k1, float k2, float k3, float k4, float k5, float k6, float p1, float p2, float f, int cx, int cy);

    // wait for core end.
    void join_client_read_thread();


private:
    // thread function
    void client_read_thread_function( );
    void image_read_thread_function( );
    void image_thread_function( );

    void disconnected();
    void image_disconnected();

    void ozcore_image_read_thread_function( );
    void ozcore_image_thread_function( );

    void ozcore_image_disconnected();

private:
    std::shared_ptr<tf::TransformListener> listener_ptr_;

    bool client_read_thread_started_;
    bool image_read_thread_started_;
    bool image_thread_started_;
    std::thread client_read_thread_;
    std::thread image_read_thread_;
    std::thread image_thread_;

    std::thread send_odo_thread_;

    bool ozcore_image_read_thread_started_;
    bool ozcore_image_thread_started_;
    std::thread ozcore_image_read_thread_;
    std::thread ozcore_image_thread_;

    std::mutex socket_access_;

    std::mutex image_socket_access_;
    std::mutex ozcore_image_socket_access_;
    int server_socket_desc_;
    int image_server_socket_desc_;
    int ozcore_image_server_socket_desc_;
    int client_socket_desc_;
    int image_socket_desc_;
    int ozcore_image_socket_desc_;
    bool client_socket_connected_;
    bool image_socket_connected_;
    bool ozcore_image_socket_connected_;
    uint64_t last_socket_activity_time_;
    uint64_t last_image_socket_activity_time_;
    uint64_t last_ozcore_image_socket_activity_time_;

    std::mutex packet_to_send_list_access_;
    std::mutex image_packet_to_send_access_;
    std::mutex ozcore_image_packet_to_send_access_;
    std::vector< BaseNaio01PacketPtr > packet_to_send_list_;
    ApiStereoCameraPacketPtr image_packet_to_send_;
    uint8_t image_buffer_to_send_[ 721920 ];
    int64_t last_image_ms_;
    int64_t last_ozcore_image_ms_;

    float actuator_position_;


    // ROS PART
    ros::Publisher velocity_pub_;
    ros::Publisher actuator_pub_;


    bool image_ready_;
};

#endif //CORE_HPP
