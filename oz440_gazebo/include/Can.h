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
#include "../include/GeoAngle.hpp"

#include <../include/oz440_api/ApiMoveActuatorPacket.hpp>
#include <oz440_api/HaOdoPacket.hpp>
#include <oz440_api/ApiGpsPacket.hpp>
#include <oz440_api/HaGpsPacket.hpp>
#include "oz440_api/ApiStatusPacket.hpp"
#include "oz440_api/HaGyroPacket.hpp"
#include "oz440_api/HaAcceleroPacket.hpp"

#ifndef SIMULATOZ_CAN_H
#define SIMULATOZ_CAN_H

class Can
{
public:

    enum CanMessageId : unsigned char
    {
        CAN_ID_GEN = 0x00,
        CAN_ID_IMU = 0x03,
        CAN_ID_GPS = 0x04,
        CAN_ID_IHM = 0x07,
        CAN_ID_VER = 0x08,
        CAN_ID_TELECO = 0x0b,
    };

    enum CanMessageType  : unsigned char
    {
        CAN_MOT_CONS = 0x00,

        CAN_IMU_ACC = 0x00,
        CAN_IMU_GYRO = 0x01,

        CAN_TELECO_KEYS = 0x01,
        CAN_TELECO_NUM_VERSION = 0x06,

        CAN_GPS_DATA = 0x00,

        CAN_IHM_LCD = 0x00,
        CAN_IHM_BUT = 0x01,

        CAN_VER_CONS = 0x02,
        CAN_VER_POS = 0x01,
    };

//  *********************************************  -- METHODES --  ****************************************************

    Can(int server_port);
    ~Can();

    void add_packet(BaseNaio01PacketPtr packet_ptr);
    void ask_stop();
    bool connected();

private:
    void init();

    void connect();
    void read_thread();
    void manage_thread();

    void manage_packet( BaseNaio01PacketPtr packet_ptr );
    void send_packet( ComSimuCanMessageId id, ComSimuCanMessageType id_msg, uint8_t data[], uint8_t len );
    void disconnect();

    void gps_manager();
    double north_bearing( double lat1, double lon1, double lat2, double lon2 );

//  *********************************************  -- ATTRIBUTS --  ****************************************************

    std::atomic<bool> stop_asked_;

    bool connect_thread_started_;
    std::thread connect_thread_;

    bool read_thread_started_;
    std::thread read_thread_;

    bool manage_thread_started_;
    std::thread manage_thread_;

    int server_port_;
    bool socket_connected_;
    int server_socket_desc_;
    int socket_desc_;
    std::mutex socket_access_;

    std::vector< BaseNaio01PacketPtr > packets_ptr_;
    std::mutex packets_access_;

    //  --  ODOMETRY  --
    bool last_odo_ticks_[4];
    HaOdoPacketPtr last_odo_packet_ptr_;

    //  --  TOOL POSITION  --
    std::mutex tool_position_access_;
    uint8_t tool_position_;
    std::mutex tool_command_access_;
    uint8_t tool_command_;

    //  --  GPS  --
    std::mutex gps_packet_access_;
    HaGpsPacketPtr gps_packet_ptr_;
    std::thread	gps_manager_thread_;
    HaGpsPacketPtr last_gps_packet_ptr_;
};

#endif //SIMULATOZ_CAN_H