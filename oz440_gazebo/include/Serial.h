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

#include <oz440_api/HaMotorsPacket.hpp>

#ifndef SIMULATOZ_SERIAL_H
#define SIMULATOZ_SERIAL_H

class Serial
{
public:
    Serial(int server_port);
    ~Serial();

    HaMotorsPacketPtr get_packet();
    void ask_stop();
    bool connected();

private:
    void init();

    void connect();
    void read_thread();
    void disconnect();

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

    HaMotorsPacketPtr packet_ptr_;
    std::mutex packet_access_;

};

#endif //SIMULATOZ_SERIAL_H