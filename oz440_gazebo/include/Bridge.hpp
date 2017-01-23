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
#include <oz440_api/HaLidarPacket.hpp>
#include <oz440_api/ApiLidarPacket.hpp>
#include <oz440_api/HaOdoPacket.hpp>
#include <oz440_api/ApiPostPacket.hpp>
#include <oz440_api/ApiGpsPacket.hpp>
#include <oz440_api/HaGpsPacket.hpp>
#include <oz440_api/ApiStereoCameraPacket.hpp>
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

    enum ComSimuCanMessageId : unsigned char
    {
        CAN_ID_GEN = 0x00,
        CAN_ID_IMU = 0x03,
        CAN_ID_GPS = 0x04,
        CAN_ID_IHM = 0x07,
        CAN_ID_VER = 0x08,
        CAN_ID_TELECO = 0x0b,
    };

    enum ComSimuCanMessageType  : unsigned char
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

    const int64_t MAIN_GRAPHIC_DISPLAY_RATE_MS = 100;
    const int64_t SERVER_SEND_COMMAND_RATE_MS = 10;
    const int64_t WAIT_SERVER_IMAGE_TIME_RATE_MS = 5;
    const int64_t IMAGE_SERVER_WATCHDOG_SENDING_RATE_MS = 100;

    const int64_t TIME_BEFORE_IMAGE_LOST_MS = 500;

    const int SIMULATOR_IMAGE_PORT = 5554;

    const int OZCORE_LIDAR_PORT = 2213;

    const int64_t COM_OZCORE_REMOTE_SEND_RATE_MS = 100;

public:

    Bridge();
    ~Bridge( );

    // launch core
    void init( bool graphical_display_on );
    void add_received_packet(BaseNaio01PacketPtr packetPtr);
    void add_received_image(BaseNaio01PacketPtr packetPtr);
    void stop_main_thread_asked();
    std::vector< BaseNaio01PacketPtr > get_packet_list_to_send();
    bool get_stop_main_thread_asked();
    bool get_image_displayer_asked();


private:
    // thread function
    void main_thread( );
    void graphic_thread( );

    //Communication with core
    void read_thread( );
    void manage_received_packet(BaseNaio01PacketPtr packetPtr);

    // images from Core to SDL functions
    void image_thread( );
    void image_preparer_thread( );
    void start_image_display();
    void stop_image_display();

    // graph
    SDL_Window *init_sdl(const char *name, int szX, int szY);

    void read_sdl_keyboard();
    bool manage_sdl_keyboard();

    void draw_robot();
    void draw_lidar( uint16_t lidar_distance_[271] );
    void draw_text( char gyro_buff[100], int x, int y );
    void draw_images( );

    // COM OZCORE

    void ozcore_read_serial_thread( );

    void ozcore_lidar_thread_function( );

    void ozcore_connect_can_thread( );
    void ozcore_send_can_packet( ComSimuCanMessageId id, ComSimuCanMessageType id_msg, uint8_t data[], uint8_t len );
    void ozcore_read_can_thread( );
    void ozcore_remote_thread();

    void disconnection_lidar();
    void disconnection_can();
    void disconnection_serial();

    int64_t get_now_ms();

    void gps_manager_thread_function( );
    double get_north_bearing( double lat1, double lon1, double lat2, double lon2 );

private:

    // Communication with Core
    bool bridge_connected_;

    bool graphical_display_on_;

    // thread part
    bool main_thread_started_;
    std::thread main_thread_;
    std::atomic<bool> stop_main_thread_asked_;

    bool stop_read_thread_asked_;
    bool read_thread_started_;
    std::thread read_thread_;

    // Packets

    concurrency::ThreadsafeQueue< BaseNaio01PacketPtr > received_packets_;
    concurrency::ThreadsafeQueue< BaseNaio01PacketPtr > received_image_;

    std::vector< BaseNaio01PacketPtr > packet_list_to_send_;
    std::mutex packet_list_to_send_access_;

    std::mutex ha_lidar_packet_ptr_access_;
    HaLidarPacketPtr ha_lidar_packet_ptr_;

    std::mutex ha_gyro_packet_ptr_access_;
    HaGyroPacketPtr ha_gyro_packet_ptr_;

    std::mutex ha_accel_packet_ptr_access_;
    HaAcceleroPacketPtr ha_accel_packet_ptr_;

    std::mutex ha_odo_packet_ptr_access;
    HaOdoPacketPtr ha_odo_packet_ptr_;

    std::mutex api_post_packet_ptr_access_;
    ApiPostPacketPtr api_post_packet_ptr_;

    std::mutex ha_gps_packet_ptr_access_;
    HaGpsPacketPtr ha_gps_packet_ptr_;
    std::thread	gps_manager_thread_;
    HaGpsPacketPtr previous_ha_gps_packet_ptr_;

    // ia part
    ControlType control_type_;

    // SDL

    int sdl_key_[SDL_NUM_SCANCODES];

    SDL_Window* screen_;
    SDL_Renderer* renderer_;

    SDL_Color sdl_color_red_;
    SDL_Color sdl_color_white_;
    TTF_Font* ttf_font_;

    uint64_t last_motor_time_;

    // LIDAR

    std::thread ozcore_lidar_thread_;
    bool ozcore_lidar_thread_started_;

    std::mutex ozcore_lidar_socket_access_;
    int ozcore_lidar_server_socket_desc_;
    int ozcore_lidar_socket_desc_;
    bool ozcore_lidar_socket_connected_;
    uint64_t last_ozcore_lidar_socket_activity_time_;

    // SERIAL

    bool ozcore_read_serial_thread_started_;
    std::thread ozcore_read_serial_thread_;

    std::mutex ozcore_serial_socket_access_;
    int ozcore_serial_server_socket_desc_;
    int ozcore_serial_socket_desc_;
    bool ozcore_serial_connected_;
    uint64_t last_ozcore_serial_socket_activity_time_;

    // CAN

    bool ozcore_can_read_thread_started_;
    std::thread ozcore_read_can_thread_;

    bool ozcore_connect_can_thread_started_;
    std::thread ozcore_connect_can_thread_;

    std::mutex ozcore_can_socket_access_;
    int ozcore_can_server_socket_desc_;
    int ozcore_can_socket_desc_;
    bool ozcore_can_socket_connected_;
    uint64_t last_ozcore_can_socket_activity_time_;

    // ODO

    bool com_ozcore_last_odo_ticks_[4];
    HaOdoPacketPtr com_ozcore_last_ha_odo_packet_ptr_;

    // REMOTE

    std::mutex com_ozcore_remote_status_access_;
    COM_OZCORE_REMOTE_STATUS com_ozcore_remote_status_;
    std::thread ozcore_remote_thread_;

    // IHM

    char com_ozcore_ihm_line_top_[ 100 ];
    char com_ozcore_ihm_line_bottom_[ 100 ];

    COM_OZCORE_IHM_BUTTON_STATUS com_ozcore_ihm_button_status_;

    // IMAGE
    bool stop_image_preparer_thread_asked_;
    bool image_prepared_thread_started_;
    std::thread image_prepared_thread_;

    bool stop_image_thread_asked_;
    bool image_thread_started_;
    std::thread image_thread_;

    uint64_t last_image_displayer_action_time_ms_;
    bool asked_image_displayer_start_;
    std::mutex simulatoz_image_actionner_access_;

    bool asked_start_video_;
    bool asked_stop_video_;

    uint64_t last_image_received_time_;
    std::mutex api_stereo_camera_packet_ptr_access_;
    ApiStereoCameraPacketPtr api_stereo_camera_packet_ptr_;
    std::mutex last_images_buffer_access_;
    uint8_t last_images_buffer_[ 4000000 ];
    ApiStereoCameraPacket::ImageType last_image_type_;

    // TOOL POSITION

    std::mutex tool_position_access_;
    uint8_t tool_position_;

    std::mutex asked_tool_position_access_;
    uint8_t asked_tool_position_;
};

#endif