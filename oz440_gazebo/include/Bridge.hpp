//=============================================================================
//
//  Copyright (C)  2014  Naio Technologies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//=============================================================================

#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <SDL2/SDL_video.h>
#include <SDL2/SDL_system.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <oz440_api/HaOdoPacket.hpp>
#include <oz440_api/ApiGpsPacket.hpp>
#include <oz440_api/HaGpsPacket.hpp>
#include "oz440_api/ApiMotorsPacket.hpp"
#include "oz440_api/ApiStatusPacket.hpp"
#include "oz440_api/HaMotorsPacket.hpp"
#include "oz440_api/HaGyroPacket.hpp"
#include "oz440_api/HaAcceleroPacket.hpp"
#include "DriverSocket.hpp"

#include "ThreadsafeQueue.hpp"


class Bridge
{
public:
    enum ControlType : uint8_t
    {
        CONTROL_TYPE_MANUAL = 0x01,
    };

    typedef struct _COM_OZCORE_REMOTE_STATUS_
    {
        bool secu_left;
        bool secu_right;

        bool arr_left;
        bool arr_right;

        bool pad_up;
        bool pad_left;
        bool pad_right;
        bool pad_down;

        bool tool_up;
        bool tool_down;

        uint8_t analog_x;
        uint8_t analog_y;

        uint8_t teleco_self_id_6;
        uint8_t teleco_act_7;
    } COM_OZCORE_REMOTE_STATUS ;


    typedef struct _COM_OZCORE_IHM_BUTTON_STATUS_
    {
        bool cancel;
        bool validate;
        bool plus;
        bool minus;
        bool right;
        bool left;
    } COM_OZCORE_IHM_BUTTON_STATUS;

    const int64_t SERVER_SEND_COMMAND_RATE_MS = 10;

    const int64_t COM_OZCORE_REMOTE_SEND_RATE_MS = 100;

public:

    Bridge();
    ~Bridge( );

    // launch core
    void init();
    void stop_main_thread_asked();
    std::vector< BaseNaio01PacketPtr > get_packet_list_to_send();
    bool get_stop_main_thread_asked();

private:


    // COM OZCORE

    void ozcore_read_serial_thread( );
    void ozcore_remote_thread();
    void disconnection_serial();

private:

    // Communication with Core
    bool bridge_connected_;

    std::atomic<bool> stop_main_thread_asked_;

    bool stop_read_thread_asked_;

    // Packets

    std::vector< BaseNaio01PacketPtr > packet_list_to_send_;
    std::mutex packet_list_to_send_access_;

    std::mutex ha_gyro_packet_ptr_access_;
    HaGyroPacketPtr ha_gyro_packet_ptr_;

    std::mutex ha_accel_packet_ptr_access_;
    HaAcceleroPacketPtr ha_accel_packet_ptr_;

    // ia part
    ControlType control_type_;

    uint64_t last_motor_time_;

    //  -- SERIAL --

    bool ozcore_read_serial_thread_started_;
    std::thread ozcore_read_serial_thread_;

    std::mutex ozcore_serial_socket_access_;
    int ozcore_serial_server_socket_desc_;
    int ozcore_serial_socket_desc_;
    bool ozcore_serial_connected_;


    // REMOTE

    std::mutex com_ozcore_remote_status_access_;
    COM_OZCORE_REMOTE_STATUS com_ozcore_remote_status_;
    std::thread ozcore_remote_thread_;

    // IHM

    char com_ozcore_ihm_line_top_[ 100 ];
    char com_ozcore_ihm_line_bottom_[ 100 ];

    COM_OZCORE_IHM_BUTTON_STATUS com_ozcore_ihm_button_status_;


};

#endif