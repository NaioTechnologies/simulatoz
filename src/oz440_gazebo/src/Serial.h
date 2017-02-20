//
// Created by fanny on 31/01/17.
//

#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include <unistd.h>
#include <net/if.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <mutex>
#include <atomic>
#include <thread>

#include "geometry_msgs/Vector3Stamped.h"

#ifndef SIMULATOZ_SERIAL_H
#define SIMULATOZ_SERIAL_H

class Serial
{
public:
    Serial(uint16_t server_port);
    ~Serial();

    void advertise( ros::NodeHandle& node );

    void cleanup();

private:
    void init();

    void connect();
    void read_thread();
    void disconnect();

    //  -- ATTRIBUTS --
    std::atomic<bool> stop_;

    std::thread connect_thread_;
    std::thread read_thread_;

    uint16_t server_port_;
    bool socket_connected_;
    int server_socket_desc_;
    int socket_desc_;
    std::mutex socket_access_;

    ros::Publisher velocity_pub_;
};

#endif //SIMULATOZ_SERIAL_H
