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

#include "HaLidarPacket.hpp"
#include "ApiLidarPacket.hpp"

#ifndef SIMULATOZ_LIDAR_H
#define SIMULATOZ_LIDAR_H

class Lidar
{
public:
    Lidar(int server_port);
    ~Lidar();

    void set_packet(HaLidarPacketPtr packet_ptr);
    void ask_stop();
    bool connected();

private:
    void init();

    void connect();
    void read_thread();
    void disconnect();
    void send_packet();

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

    HaLidarPacketPtr packet_ptr_;
    std::mutex packet_access_;

    uint16_t nbMesures_;
    uint16_t nbTelegrammes_;

};

#endif //SIMULATOZ_LIDAR_H
