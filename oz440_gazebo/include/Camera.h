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



#ifndef SIMULATOZ_CAMERA_H
#define SIMULATOZ_CAMERA_H

class Camera
{
public:
    Camera(int server_port);
    ~Camera();

    void set_image(std::array < uint8_t, 721920 > image);
    void ask_stop();
    bool connected();

private:
    void init();

    void connect();
    void disconnect();
    void send_image();

    //  -- ATTRIBUTS --
    std::atomic<bool> stop_asked_;

    bool connect_thread_started_;
    std::thread connect_thread_;

    int server_port_;
    bool socket_connected_;
    int server_socket_desc_;
    int socket_desc_;

    std::mutex image_access_;

    std::array < uint8_t, 721920 > image_;

};

#endif //SIMULATOZ_CAMERA_H