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
#include <HaLidarPacket.hpp>
#include <ApiLidarPacket.hpp>
#include <HaOdoPacket.hpp>
#include <ApiPostPacket.hpp>
#include <ApiGpsPacket.hpp>
#include <HaGpsPacket.hpp>
#include <ApiStereoCameraPacket.hpp>

#include "ApiCodec/Naio01Codec.hpp"
#include "ApiCodec/ApiMotorsPacket.hpp"
#include "ApiCodec/ApiStatusPacket.hpp"
#include "ApiCodec/HaMotorsPacket.hpp"
#include "ApiCodec/HaGyroPacket.hpp"
#include "ApiCodec/HaAcceleroPacket.hpp"
#include "DriverSocket.hpp"


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

    typedef struct _COM_SIMU_REMOTE_STATUS_
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
    } COM_SIMU_REMOTE_STATUS ;


    typedef struct _COM_SIMU_IHM_BUTTON_STATUS_
    {
        bool cancel;
        bool validate;
        bool plus;
        bool minus;
        bool right;
        bool left;
    } COM_SIMU_IHM_BUTTON_STATUS;

    const int64_t MAIN_GRAPHIC_DISPLAY_RATE_MS = 100;
    const int64_t SERVER_SEND_COMMAND_RATE_MS = 10;
    const int64_t WAIT_SERVER_IMAGE_TIME_RATE_MS = 5;
    const int64_t IMAGE_SERVER_WATCHDOG_SENDING_RATE_MS = 100;

    const int64_t TIME_BEFORE_IMAGE_LOST_MS = 500;

    const int SIMULATOR_IMAGE_PORT = 5554;

    const int OZCORE_LIDAR_PORT = 2213;

    const int64_t COM_SIMU_REMOTE_SEND_RATE_MS = 100;

public:

    Bridge( );
    ~Bridge( );

    // launch core
    void init( bool graphical_display_on, std::string can );
    void set_received_packet(BaseNaio01PacketPtr packetPtr);
    void set_received_image(BaseNaio01PacketPtr packetPtr);
    std::vector< BaseNaio01PacketPtr > get_packet_list_to_send();
    void clear_packet_list_to_send();
    bool get_com_simu_can_connected_();

private:
    // thread function
    void main_thread( );
    void graphic_thread( );

    //Communication with core
    void read_thread( );
    void write_thread( );

    // images from Core to SDL thread functions
    void image_thread( );
    void image_preparer_thread( );

    // communication
    void manage_received_packet(BaseNaio01PacketPtr packetPtr);

    // graph
    SDL_Window *init_sdl(const char *name, int szX, int szY);

    void read_sdl_keyboard();
    bool manage_sdl_keyboard();

    void draw_robot();
    void draw_lidar( uint16_t lidar_distance_[271] );
    void draw_text( char gyro_buff[100], int x, int y );
    void draw_images( );

    // COM SIMU
    void com_simu_create_virtual_can( );
    void com_simu_create_serial_thread_function( );
    void com_simu_read_serial_thread_function( );
    void com_simu_lidar_to_core_thread_function( );
    void com_simu_connect_can( );
    void com_simu_transform_and_write_to_can( BaseNaio01PacketPtr packetPtr );
    void com_simu_send_can_packet( ComSimuCanMessageId id, ComSimuCanMessageType id_msg, uint8_t data[], uint8_t len );

    void com_simu_remote_thread_function( );

    void com_simu_read_can_thread_function( );

    void send_remote_can_packet( ComSimuCanMessageType message_type );

    void send_keypad_can_packet( );

    int64_t get_now_ms();

    void image_displayer_starter_thread_function();
    void start_image_display();
    void stop_image_display();

    void text_keyboard_reader_thread_function( );

    void gps_manager_thread_function( );

    double get_north_bearing( double lat1, double lon1, double lat2, double lon2 );

public:

    bool stop_main_thread_asked_;

private:

    // Communication with Core
    std::vector<BaseNaio01PacketPtr> received_packets_;
    std::mutex received_packets_access_;

    BaseNaio01PacketPtr received_image_;
    std::mutex received_image_access_;

    std::vector< BaseNaio01PacketPtr > packet_list_to_send_;
    std::mutex packet_list_to_send_access_;

    // Can
    bool use_virtual_can_;
    std::string can_;
    bool graphical_display_on_;

    // thread part
    bool main_thread_started_;
    std::thread main_thread_;

    bool stop_read_thread_asked_;
    bool read_thread_started_;
    std::thread ReadThread_;

    bool stop_write_thread_asked_;
    bool write_thread_started_;
    std::thread write_thread_;

    // sdl part
    int sdl_key_[SDL_NUM_SCANCODES];

    // Packets
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
    HaGpsPacketPtr previous_ha_gps_packet_ptr_;

    std::mutex api_stereo_camera_packet_ptr_access_;
    ApiStereoCameraPacketPtr api_stereo_camera_packet_ptr_;
    std::mutex last_images_buffer_access_;
    uint8_t last_images_buffer_[ 4000000 ];
    ApiStereoCameraPacket::ImageType last_image_type_;

    // ia part
    ControlType control_type_;

    SDL_Window* screen_;
    SDL_Renderer* renderer_;

    SDL_Color sdl_color_red_;
    SDL_Color sdl_color_white_;
    TTF_Font* ttf_font_;

    bool asked_start_video_;
    bool asked_stop_video_;

    std::thread image_prepared_thread_;

    uint64_t last_motor_time_;

    bool stop_image_thread_asked_;
    bool image_thread_started_;
    std::thread image_thread_;

    uint64_t last_image_received_time_;

    // COM SIMU
    std::thread com_simu_create_serial_thread_;
    std::thread com_simu_read_serial_thread_;
    std::thread com_simu_lidar_to_core_thread_;

    std::mutex com_simu_can_socket_access_;
    SOCKET com_simu_can_socket_;

    bool com_simu_can_connected_;

    bool com_simu_last_odo_ticks_[4];
    HaOdoPacketPtr com_simu_last_ha_odo_packet_ptr_;

    std::mutex com_simu_remote_status_access_;
    COM_SIMU_REMOTE_STATUS com_simu_remote_status_;

    std::thread com_simu_remote_thread_;

    char com_simu_ihm_line_top_[ 100 ];
    char com_simu_ihm_line_bottom_[ 100 ];

    std::thread com_simu_read_can_thread_;

    COM_SIMU_IHM_BUTTON_STATUS com_simu_ihm_button_status_;

    bool com_simu_serial_connected_;

    std::mutex image_socket_desc_access_;

    bool display_simuloz_camera_;

    bool stop_image_preparer_thread_asked_;
    bool image_prepared_thread_started_;

    uint64_t last_image_displayer_action_time_ms_;
    std::thread	image_displayer_starter_thread_;
    bool asked_image_displayer_start_;
    std::mutex simulatoz_image_actionner_access_;

    std::thread	text_keyboard_reader_thread_;

    uint64_t last_text_keyboard_hit_time_;

    std::thread	gps_manager_thread_;

    std::mutex tool_position_access_;
    uint8_t tool_position_;

    std::mutex asked_tool_position_access_;
    uint8_t asked_tool_position_;
};

#endif