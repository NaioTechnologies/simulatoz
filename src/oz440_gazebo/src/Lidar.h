//
// Created by fanny on 31/01/17.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <net/if.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <mutex>
#include <atomic>
#include <thread>

#ifndef SIMULATOZ_LIDAR_H
#define SIMULATOZ_LIDAR_H

class Lidar
{
public:
    Lidar(int server_port);
    ~Lidar();

    void subscribe( ros::NodeHandle& node);
    void cleanup();

private:
    void init();

    void connect();
    void read_thread();

    void callback_lidar( const sensor_msgs::LaserScan::ConstPtr& lidar_msg );
    void disconnect();

    void createTrame(int dist[271] , int albedo[271], char trame[10000],uint64_t nbMesures,uint64_t nbTelegrammes,struct timespec timeInit);
    long elapsedMillis(struct timespec dateDepart);

        //  -- ATTRIBUTS --
    std::atomic<bool> stop_asked_;

    bool connect_thread_started_;
    std::thread connect_thread_;

    bool read_thread_started_;
    std::thread read_thread_;

    int server_port_;
    bool socket_connected_;
    int server_socket_desc_;
    int socket_desc_;
    std::mutex socket_access_;

    uint16_t nbMesures_;
    uint16_t nbTelegrammes_;

    ros::Subscriber lidar_sub_;
    struct timespec timeInit;
};

#endif //SIMULATOZ_LIDAR_H
