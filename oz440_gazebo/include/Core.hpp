#ifndef CORE_HPP
#define CORE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <array>

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

    void ozcore_image_read_thread_function( );
    void ozcore_image_thread_function( );

    void disconnection_ozcore_image();

private:

    std::shared_ptr<tf::TransformListener> listener_ptr_;
    std::shared_ptr<Bridge> bridge_ptr_;

    std::vector< BaseNaio01PacketPtr > received_packet_list_;

    concurrency::ThreadsafeQueue< std::array<uint8_t, 721920 > >  ozcore_image_to_send_;

    bool read_thread_started_;
    std::thread read_thread_;

    std::thread send_odo_thread_;

    bool test_thread_started_;
    std::thread test_thread_;

    bool bridge_thread_started_;
    std::thread bridge_thread_;
    bool graphics_on_;

    bool ozcore_image_read_thread_started_;
    bool ozcore_image_thread_started_;
    std::thread ozcore_image_read_thread_;
    std::thread ozcore_image_thread_;

    std::mutex ozcore_image_socket_access_;
    int ozcore_image_server_socket_desc_;
    int ozcore_image_socket_desc_;
    bool ozcore_image_socket_connected_;
    uint64_t last_ozcore_image_socket_activity_time_;

    float actuator_position_;

    // ROS PART
    ros::Publisher velocity_pub_;
    ros::Publisher actuator_pub_;

};

#endif //CORE_HPP
