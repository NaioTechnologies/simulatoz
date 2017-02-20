//==================================================================================================
//
//  Copyright(c)  2016  Naïo Technologies
//
//  These coded instructions, statements, and computer programs contain unpublished proprietary
//  information written by Naio Technologies and are protected by copyright law. They may not be
//  disclosed to third parties or copied or duplicated in any form, in whole or in part, without
//  the prior written consent of Naio Technologies.
//
//==================================================================================================

#ifndef SIMULATOZ_CAMERA_H
#define SIMULATOZ_CAMERA_H

//==================================================================================================
// I N C L U D E   F I L E S

#include <chrono>
#include <unistd.h>
#include <net/if.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <mutex>
#include <atomic>
#include <thread>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class Camera
{
public:
    Camera( uint16_t server_port);
    ~Camera();

    void cleanup();
    void subscribe( ros::NodeHandle& node );

private:
    void init();

    static constexpr std::size_t buffer_size_{ 721920 };

    void connect();
    void disconnect();
    void send_image( std::array< uint8_t, buffer_size_ > image );

    void callback_camera( const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right );

    //  -- ATTRIBUTS --
    std::atomic<bool> stop_;

    bool connect_thread_started_;
    std::thread connect_thread_;

    uint16_t server_port_;
    bool socket_connected_;
    int server_socket_desc_;
    int socket_desc_;

    message_filters::Subscriber< sensor_msgs::Image > image_left_sub_;
    message_filters::Subscriber< sensor_msgs::Image > image_right_sub_;
    message_filters::TimeSynchronizer< sensor_msgs::Image, sensor_msgs::Image > sync;
};

#endif //SIMULATOZ_CAMERA_H