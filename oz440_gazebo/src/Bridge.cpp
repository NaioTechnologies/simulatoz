
#include "../include/Bridge.hpp"

#include "ros/ros.h"

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <unistd.h>
#include <net/if.h>
#include <sys/fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <pthread.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/syscall.h>


#include <../include/oz440_api/ApiMoveActuatorPacket.hpp>
#include <../include/oz440_api/ApiCommandPacket.hpp>
#include <../include/oz440_api/HaAcceleroPacket.hpp>
#include <../include/oz440_api/ApiWatchdogPacket.hpp>
#include <zlib.h>

// com_ozcore
#include "../include/DriverSerial.hpp"
#include "../include/DriverSocket.hpp"
#include "../include/GeoAngle.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

static pid_t gettid( void )
{
    return syscall( __NR_gettid );
}

// #####################################################################################################################
// ---------------------------------------------------------------------------------------------------------------------
// #####################################################################################################################

Bridge::Bridge( ) :
        stop_main_thread_asked_{ false },
        read_thread_started_{ false },
        packet_list_to_send_{ },
        ha_odo_packet_ptr_{ nullptr },
        ha_gps_packet_ptr_{ nullptr },
        control_type_{ ControlType::CONTROL_TYPE_MANUAL },
        last_motor_time_{ 0L },
        received_packets_{},
        ozcore_serial_connected_{false},
        ozcore_can_socket_connected_{false},
        bridge_connected_{false},
        last_gyro_packet_send_{0}
{

    com_ozcore_last_odo_ticks_[0] = false;
    com_ozcore_last_odo_ticks_[1] = false;
    com_ozcore_last_odo_ticks_[2] = false;
    com_ozcore_last_odo_ticks_[3] = false;

    com_ozcore_remote_status_.pad_down = false;
    com_ozcore_remote_status_.pad_up = false;
    com_ozcore_remote_status_.pad_left = false;
    com_ozcore_remote_status_.pad_right = false;
    com_ozcore_remote_status_.analog_x = 63;
    com_ozcore_remote_status_.analog_y = 63;

    com_ozcore_remote_status_.tool_down = false;
    com_ozcore_remote_status_.tool_up = false;

    com_ozcore_remote_status_.teleco_self_id_6 = 255;
    com_ozcore_remote_status_.teleco_act_7 = 253;

    for( uint i = 0 ; i < 20 ; i++ )
    {
        com_ozcore_ihm_line_top_[ i ] = ' ';
        com_ozcore_ihm_line_bottom_[ i ] = ' ';
    }

    for( uint i = 20 ; i < 100 ; i++ )
    {
        com_ozcore_ihm_line_top_[ i ] = '\0';
        com_ozcore_ihm_line_bottom_[ i ] = '\0';
    }

    com_ozcore_ihm_button_status_.cancel = false;
    com_ozcore_ihm_button_status_.validate = false;
    com_ozcore_ihm_button_status_.minus = false;
    com_ozcore_ihm_button_status_.plus = false;
    com_ozcore_ihm_button_status_.left = false;
    com_ozcore_ihm_button_status_.right = false;

}

// ##################################################################################################

Bridge::~Bridge( )
{

}

// ##################################################################################################

// Starts most threads

void Bridge::init( bool graphical_display_on)
{
    try
    {
        uint64_t last_screen_output_time = 0;
        uint64_t last_key_time = 0;

        graphical_display_on_ = graphical_display_on;

        read_thread_ = std::thread(&Bridge::read_thread, this);

        std::this_thread::sleep_for(1500ms);

        ozcore_read_serial_thread_ = std::thread(&Bridge::ozcore_read_serial_thread, this);

        ozcore_read_can_thread_ = std::thread(&Bridge::ozcore_read_can_thread, this);
        ozcore_remote_thread_ = std::thread(&Bridge::ozcore_remote_thread, this);

        gps_manager_thread_ = std::thread(&Bridge::gps_manager_thread_function, this);

        bridge_connected_ = true;

        // creates graphic thread
        if (graphical_display_on_)
        {
            graphic_thread();
        }

        read_thread_.join();
        ozcore_read_serial_thread_.join();
        ozcore_read_can_thread_.join();
        ozcore_remote_thread_.join();
        gps_manager_thread_.join();

        ROS_ERROR("End of bridge thread");
    }
    catch ( std::exception e ) {
        std::cout<<"Exception init catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

// Enables Core to add a received packet

void Bridge::add_received_packet(BaseNaio01PacketPtr packetPtr)
{
    received_packets_.emplace(std::move(packetPtr));
}

// ##################################################################################################

// Ask to stop main thread

void Bridge::stop_main_thread_asked()
{
    stop_main_thread_asked_ = true;
}

// ##################################################################################################

// Enables Core to get motor and actuator orders

std::vector< BaseNaio01PacketPtr > Bridge::get_packet_list_to_send()
{
    packet_list_to_send_access_.lock();

    std::vector< BaseNaio01PacketPtr > list = packet_list_to_send_;
    packet_list_to_send_.clear();

    packet_list_to_send_access_.unlock();

    return( list );
}

// ##################################################################################################

bool Bridge::get_stop_main_thread_asked()
{
    return(stop_main_thread_asked_);
}

// ##################################################################################################

bool Bridge::get_can_connected_()
{
    return(ozcore_can_socket_connected_);
}
// ##################################################################################################

// thread function : read from simulator socket and calls manage_received_packets

void Bridge::read_thread( )
{
    try{

        ROS_INFO("Starting read thread !");

        read_thread_started_ = true;
        BaseNaio01PacketPtr packetPtr;

        while(!stop_main_thread_asked_)
        {
            received_packets_.wait_and_pop(packetPtr);
            manage_received_packet(packetPtr);
        }

        read_thread_started_ = false;

        ROS_INFO("Stopping read thread !");

    }
    catch ( std::exception e )
    {
        std::cout<<"Exception server_read_thread catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

// Manages all the packets received from Core : gyro, accelero, odo, post, Gps, sends actuator position to OzCore (via CAN)

void Bridge::manage_received_packet(BaseNaio01PacketPtr packetPtr)
{
    try {

        if (std::dynamic_pointer_cast<HaGyroPacket>(packetPtr))
        {
            HaGyroPacketPtr haGyroPacketPtr = std::dynamic_pointer_cast<HaGyroPacket>(packetPtr);

            ha_gyro_packet_ptr_access_.lock();
            ha_gyro_packet_ptr_ = haGyroPacketPtr;
            ha_gyro_packet_ptr_access_.unlock();

            uint8_t data[ 6 ];

            data[ 0 ] = ( haGyroPacketPtr->x >> 8 ) & 0xFF;
            data[ 1 ] = ( haGyroPacketPtr->x >> 0 ) & 0xFF;

            data[ 2 ] = ( haGyroPacketPtr->y >> 8 ) & 0xFF;
            data[ 3 ] = ( haGyroPacketPtr->y >> 0 ) & 0xFF;

            data[ 4 ] = ( haGyroPacketPtr->z >> 8 ) & 0xFF;
            data[ 5 ] = ( haGyroPacketPtr->z >> 0 ) & 0xFF;

            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_IMU, ComSimuCanMessageType::CAN_IMU_GYRO, data, 6 );

//            uint64_t now = get_now_ms();
//
//            int duree_entre_deux_packets = (int)now - last_gyro_packet_send_ ;
//
//            if(duree_entre_deux_packets > 15){
//                ROS_ERROR("Duree %d", duree_entre_deux_packets);
//            }
//
//            last_gyro_packet_send_ = now;
        }
        else if (std::dynamic_pointer_cast<HaAcceleroPacket>(packetPtr))
        {
            HaAcceleroPacketPtr haAcceleroPacketPtr = std::dynamic_pointer_cast<HaAcceleroPacket>(packetPtr);

            ha_accel_packet_ptr_access_.lock();
            ha_accel_packet_ptr_ = haAcceleroPacketPtr;
            ha_accel_packet_ptr_access_.unlock();

            uint8_t data[ 6 ];

            data[ 0 ] = ( haAcceleroPacketPtr->x >> 8 ) & 0xFF;
            data[ 1 ] = ( haAcceleroPacketPtr->x >> 0 ) & 0xFF;

            data[ 2 ] = ( haAcceleroPacketPtr->y >> 8 ) & 0xFF;
            data[ 3 ] = ( haAcceleroPacketPtr->y >> 0 ) & 0xFF;

            data[ 4 ] = ( haAcceleroPacketPtr->z >> 8 ) & 0xFF;
            data[ 5 ] = ( haAcceleroPacketPtr->z >> 0 ) & 0xFF;

            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_IMU, ComSimuCanMessageType::CAN_IMU_ACC, data, 6 );
        }
        else if (std::dynamic_pointer_cast<HaOdoPacket>(packetPtr))
        {
            HaOdoPacketPtr haOdoPacketPtr = std::dynamic_pointer_cast<HaOdoPacket>(packetPtr);

            ha_odo_packet_ptr_access.lock();
            ha_odo_packet_ptr_ = haOdoPacketPtr;
            ha_odo_packet_ptr_access.unlock();

            if( com_ozcore_last_ha_odo_packet_ptr_ != nullptr )
            {
                if( com_ozcore_last_ha_odo_packet_ptr_->fr != haOdoPacketPtr->fr )
                {
                    com_ozcore_last_odo_ticks_[ 0 ] = not com_ozcore_last_odo_ticks_[ 0 ];
                }

                if( com_ozcore_last_ha_odo_packet_ptr_->fl != haOdoPacketPtr->fl )
                {
                    com_ozcore_last_odo_ticks_[ 1 ] = not com_ozcore_last_odo_ticks_[ 1 ];
                }

                if( com_ozcore_last_ha_odo_packet_ptr_->rr != haOdoPacketPtr->rr )
                {
                    com_ozcore_last_odo_ticks_[ 2 ] = not com_ozcore_last_odo_ticks_[ 2 ];
                }

                if( com_ozcore_last_ha_odo_packet_ptr_->rl != haOdoPacketPtr->rl )
                {
                    com_ozcore_last_odo_ticks_[ 3 ] = not com_ozcore_last_odo_ticks_[ 3 ];
                }
            }

            uint8_t data[ 1 ];

            data[ 0 ] = 0x00;

            if( com_ozcore_last_odo_ticks_[ 0 ] )
            {
                data[ 0 ] = ( data[ 0 ] | ( 0x01 << 0 ) );
            }

            if( com_ozcore_last_odo_ticks_[ 1 ] )
            {
                data[ 0 ] = ( data[ 0 ] | ( 0x01 << 1 ) );
            }

            if( com_ozcore_last_odo_ticks_[ 2 ] )
            {
                data[ 0 ] = ( data[ 0 ] | ( 0x01 << 2 ) );
            }

            if( com_ozcore_last_odo_ticks_[ 3 ] )
            {
                data[ 0 ] = ( data[ 0 ] | ( 0x01 << 3 ) );
            }

            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_GEN, ComSimuCanMessageType::CAN_MOT_CONS, data, 1 );

            com_ozcore_last_ha_odo_packet_ptr_ = haOdoPacketPtr;
        }
        else if (std::dynamic_pointer_cast<HaGpsPacket>(packetPtr))
        {
            HaGpsPacketPtr haGpsPacketPtr = std::dynamic_pointer_cast<HaGpsPacket>(packetPtr);

            ha_gps_packet_ptr_access_.lock();
            previous_ha_gps_packet_ptr_ = ha_gps_packet_ptr_;
            ha_gps_packet_ptr_ = haGpsPacketPtr;
            ha_gps_packet_ptr_access_.unlock();
        }
        else if (std::dynamic_pointer_cast<ApiMoveActuatorPacket>(packetPtr))
        {
            ApiMoveActuatorPacketPtr api_move_actuator_packet_ptr = std::dynamic_pointer_cast<ApiMoveActuatorPacket>(
                    packetPtr);

            tool_position_access_.lock();
            tool_position_ = api_move_actuator_packet_ptr->position;
            tool_position_access_.unlock();

            uint8_t data[1];

            tool_position_access_.lock();
            data[0] = tool_position_;
            tool_position_access_.unlock();

            ozcore_send_can_packet(ComSimuCanMessageId::CAN_ID_VER, ComSimuCanMessageType::CAN_VER_POS, data, 1);
        }

    }
    catch ( std::exception e ) {
        std::cout<<"Exception main_thread catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

void Bridge::graphic_thread( )
{
    try{

        ROS_INFO("Starting graphic_thread.");

        for ( int i = 0 ; i < SDL_NUM_SCANCODES ; i++ )
        {
            sdl_key_[i] = 0;
        }

        // create graphics
        screen_ = init_sdl("Simulatoz Bridge", 800, 730);

        while( !stop_main_thread_asked_)
        {
            read_sdl_keyboard();
            manage_sdl_keyboard();

            // drawing part.
            SDL_SetRenderDrawColor( renderer_, 0, 0, 0, 255 ); // the rect color (solid red)
            SDL_Rect background;
            background.w = 800;
            background.h = 483;
            background.y = 0;
            background.x = 0;

            SDL_RenderFillRect( renderer_, &background );

            draw_text( com_ozcore_ihm_line_top_, 500, 410 );
            draw_text( com_ozcore_ihm_line_bottom_, 500, 420 );

            // ##############################################

            SDL_RenderPresent( renderer_ );

            // repeat keyboard reading for smoother command inputs
            read_sdl_keyboard();
            manage_sdl_keyboard();

            std::this_thread::sleep_for( std::chrono::milliseconds( 50) );

        }

        SDL_Quit();

    }
    catch ( std::exception e ) {
        std::cout<<"Exception graphic_thread catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

void Bridge::draw_text( char buffer[100], int x, int y )
{
    try {
        SDL_Surface *surfaceMessageAccel = TTF_RenderText_Solid(ttf_font_, buffer, sdl_color_white_);

        SDL_Texture *messageAccel = SDL_CreateTextureFromSurface(renderer_, surfaceMessageAccel);

        SDL_FreeSurface(surfaceMessageAccel);

        SDL_Rect message_rect_accel;
        message_rect_accel.x = x;
        message_rect_accel.y = y;

        SDL_QueryTexture(messageAccel, NULL, NULL, &message_rect_accel.w, &message_rect_accel.h);
        SDL_RenderCopy(renderer_, messageAccel, NULL, &message_rect_accel);

        SDL_DestroyTexture(messageAccel);
    }
    catch ( std::exception e )
    {
        std::cout<<"Exception draw_text catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

SDL_Window* Bridge::init_sdl(const char *name, int szX, int szY)
{
    try {
        ROS_INFO("Init SDL");

        SDL_Window *screen;
        std::cout << ".";

        SDL_Init(SDL_INIT_EVERYTHING);
        std::cout << ".";

        screen = SDL_CreateWindow(name, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, szX, szY, SDL_WINDOW_SHOWN);
        std::cout << ".";

        renderer_ = SDL_CreateRenderer(screen, 0, SDL_RENDERER_ACCELERATED);
        std::cout << ".";

        TTF_Init();
        std::cout << ".";

        // Set render color to black ( background will be rendered in this color )
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
        std::cout << ".";

        SDL_RenderClear(renderer_);
        std::cout << ".";

        sdl_color_red_ = {255, 0, 0, 0};
        sdl_color_white_ = {255, 255, 255, 0};
        ttf_font_ = TTF_OpenFont("mono.ttf", 12);

        if (ttf_font_ == nullptr) {
            std::cerr << "Failed to load SDL Font! Error: " << TTF_GetError() << '\n';
        }

        ROS_INFO("DONE" );

        return screen;

    }
    catch ( std::exception e ) {
        std::cout<<"Exception init_sdl catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

void
Bridge::read_sdl_keyboard()
{
    try {

        SDL_Event event;

        while (SDL_PollEvent(&event)  and ros :: ok() ) {
            switch (event.type) {
                // Cas d'une touche enfoncée
                case SDL_KEYDOWN:
                    sdl_key_[event.key.keysym.scancode] = 1;
                    break;
                    // Cas d'une touche relâchée
                case SDL_KEYUP:
                    sdl_key_[event.key.keysym.scancode] = 0;
                    break;
            }
        }
    }
    catch ( std::exception e ) {
        std::cout<<"Exception main_thread catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

bool
Bridge::manage_sdl_keyboard()
{
    try {
        bool keyPressed = false;

        int8_t left = 0;
        int8_t right = 0;

        if (sdl_key_[SDL_SCANCODE_ESCAPE] == 1) {
            stop_main_thread_asked_ = true;

            bridge_connected_ = false;

            return true;
        }

        if (sdl_key_[SDL_SCANCODE_UP] == 1 and sdl_key_[SDL_SCANCODE_LEFT] == 1) {
            com_ozcore_remote_status_.analog_x = 250;
            com_ozcore_remote_status_.analog_y = 5;

            left = 32;
            right = 63;
            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_UP] == 1 and sdl_key_[SDL_SCANCODE_RIGHT] == 1) {
            com_ozcore_remote_status_.analog_x = 250;
            com_ozcore_remote_status_.analog_y = 250;

            left = 63;
            right = 32;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_DOWN] == 1 and sdl_key_[SDL_SCANCODE_LEFT] == 1) {
            com_ozcore_remote_status_.analog_x = 5;
            com_ozcore_remote_status_.analog_y = 5;

            left = -32;
            right = -63;
            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_DOWN] == 1 and sdl_key_[SDL_SCANCODE_RIGHT] == 1) {
            com_ozcore_remote_status_.analog_x = 5;
            com_ozcore_remote_status_.analog_y = 250;

            left = -63;
            right = -32;
            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_UP] == 1) {

            com_ozcore_remote_status_.analog_x = 250;

            left = 63;
            right = 63;
            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_DOWN] == 1) {
            com_ozcore_remote_status_.analog_x = 5;

            left = -63;
            right = -63;
            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_PLUS] == 1) {
            com_ozcore_remote_status_.tool_up = true;
            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_MINUS] == 1) {
            com_ozcore_remote_status_.tool_down = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_LEFT] == 1) {
            com_ozcore_remote_status_.analog_y = 5;

            left = -63;
            right = 63;
            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_RIGHT] == 1) {
            com_ozcore_remote_status_.analog_y = 250;

            left = 63;
            right = -63;
            keyPressed = true;
        }

        // ########################
        //         com_ozcore
        // ########################

        if (sdl_key_[SDL_SCANCODE_KP_7] == 1) {
            com_ozcore_remote_status_.secu_left = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_9] == 1) {
            com_ozcore_remote_status_.secu_right = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_1] == 1) {
            com_ozcore_remote_status_.arr_left = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_3] == 1) {
            com_ozcore_remote_status_.arr_right = true;

            keyPressed = true;
        }


        if (sdl_key_[SDL_SCANCODE_KP_4] == 1) {
            com_ozcore_remote_status_.pad_left = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_6] == 1) {
            com_ozcore_remote_status_.pad_right = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_8] == 1) {
            com_ozcore_remote_status_.pad_up = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_KP_2] == 1) {
            com_ozcore_remote_status_.pad_down = true;

            keyPressed = true;
        }

        if (sdl_key_[SDL_SCANCODE_PAGEUP] == 1) {
            com_ozcore_ihm_button_status_.plus = true;

            keyPressed = true;
        } else {
            com_ozcore_ihm_button_status_.plus = false;
        }

        if (sdl_key_[SDL_SCANCODE_PAGEDOWN] == 1) {
            com_ozcore_ihm_button_status_.minus = true;

            keyPressed = true;
        } else {
            com_ozcore_ihm_button_status_.minus = false;
        }

        if (sdl_key_[SDL_SCANCODE_HOME] == 1) {
            com_ozcore_ihm_button_status_.left = true;

            keyPressed = true;
        } else {
            com_ozcore_ihm_button_status_.left = false;
        }

        if (sdl_key_[SDL_SCANCODE_END] == 1) {
            com_ozcore_ihm_button_status_.right = true;

            keyPressed = true;
        } else {
            com_ozcore_ihm_button_status_.right = false;
        }

        if (sdl_key_[SDL_SCANCODE_INSERT] == 1) {
            com_ozcore_ihm_button_status_.validate = true;

            keyPressed = true;
        } else {
            com_ozcore_ihm_button_status_.validate = false;
        }

        if (sdl_key_[SDL_SCANCODE_DELETE] == 1) {
            com_ozcore_ihm_button_status_.cancel = true;

            keyPressed = true;
        } else {
            com_ozcore_ihm_button_status_.cancel = false;
        }

        // #######################

        if (sdl_key_[SDL_SCANCODE_LEFT] == 0 and sdl_key_[SDL_SCANCODE_RIGHT] == 0) {
            com_ozcore_remote_status_.analog_y = 128;
        }

        if (sdl_key_[SDL_SCANCODE_UP] == 0 and sdl_key_[SDL_SCANCODE_DOWN] == 0) {
            com_ozcore_remote_status_.analog_x = 128;
        }

        if (sdl_key_[SDL_SCANCODE_KP_PLUS] == 0) {
            com_ozcore_remote_status_.tool_up = false;
        }

        if (sdl_key_[SDL_SCANCODE_KP_MINUS] == 0) {
            com_ozcore_remote_status_.tool_down = false;
        }

        if (sdl_key_[SDL_SCANCODE_KP_6] == 0) {
            com_ozcore_remote_status_.pad_right = false;
        }

        if (sdl_key_[SDL_SCANCODE_KP_8] == 0) {
            com_ozcore_remote_status_.pad_up = false;
        }

        if (sdl_key_[SDL_SCANCODE_KP_2] == 0) {
            com_ozcore_remote_status_.pad_down = false;
        }

        if (sdl_key_[SDL_SCANCODE_KP_4] == 0) {
            com_ozcore_remote_status_.pad_left = false;
        }

        return keyPressed;

    }
    catch ( std::exception e ) {
        std::cout<<"Exception SDL keyboard catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

// Reads motor orders from OzCore on the Serial and add them to packet_list_to_send

void Bridge::ozcore_read_serial_thread( )
{
    try {

        using namespace std::chrono_literals;

        unsigned char b[200];

        int motorNumber;
        int posInEntete = 0;
        char motors[ 3 ] = { 0 };

        ozcore_read_serial_thread_started_ = true;
        ozcore_serial_connected_ = false;

        ozcore_serial_server_socket_desc_ = DriverSocket::openSocketServer(5554);

        char received_buffer[4096];

        while (!stop_main_thread_asked_)
        {
            if (not ozcore_serial_connected_ and ozcore_serial_server_socket_desc_ > 0)
            {
                ozcore_serial_socket_desc_ = DriverSocket::waitConnectTimer(ozcore_serial_server_socket_desc_, stop_main_thread_asked_);

                std::this_thread::sleep_for(50ms);

                if (ozcore_serial_socket_desc_ > 0)
                {
                    ozcore_serial_connected_ = true;

                    ROS_ERROR("Serial connected port 5554");
                }
            }

            if (ozcore_serial_connected_)
            {
                ozcore_serial_socket_access_.lock();
                ssize_t size = read(ozcore_serial_socket_desc_, b, 1);
                ozcore_serial_socket_access_.unlock();

                if(size < 0)
                {
                    if( errno == 32 ){
                        disconnection_serial();
                    }
                }
                else if ( size == 0){
                    if( errno == 11 or errno == 32 ){
                        disconnection_serial();
                    }
                }
                else if (size > 0){

                    if ( posInEntete == 2 )
                    {
                        motors[ motorNumber ] = ( ( ( char ) b[ 0 ] ) * 2 ) - 128;

                        if ( motorNumber == 2 )
                        {
                            HaMotorsPacketPtr haMotorsPacketPtr = std::make_shared<HaMotorsPacket>( motors[ 2 ], motors[ 1 ] );

                            packet_list_to_send_access_.lock();
                            packet_list_to_send_.push_back( haMotorsPacketPtr );
                            packet_list_to_send_access_.unlock();
                        }

                        posInEntete = 0;
                    }

                    if ( posInEntete == 1 )
                    {
                        if ( b[0] == 6 )
                        {
                            motorNumber = 1;
                            posInEntete = 2;
                        }
                        else if ( b[0] == 7 )
                        {
                            motorNumber = 2;
                            posInEntete = 2;
                        }
                        else
                        {
                            posInEntete = 0;
                        }
                    }

                    if ( ( posInEntete == 0 ) && ( b[ 0 ] == 128 ) )
                    {
                        posInEntete = 1;
                    }
                }
            }

            std::this_thread::sleep_for(5ms);
        }

        close(ozcore_serial_server_socket_desc_);

        ozcore_read_serial_thread_started_ = false;

        disconnection_serial();
    }
    catch (std::exception e)
    {
        std::cout << "Exception ozcore_serial_thread_function catch : " << e.what() << std::endl;
    }

}

// ##################################################################################################

void Bridge::ozcore_send_can_packet( ComSimuCanMessageId id, ComSimuCanMessageType id_msg, uint8_t data[], uint8_t len )
{
    try {

        if (not ozcore_can_socket_connected_)
        {
            return;
        }

        struct can_frame frame;
        ssize_t nbytes = -1;
        int nbTests = 0;

        frame.can_id = (unsigned int) (id * 128 + id_msg);
        frame.can_dlc = len;

        for (uint8_t i = 0; i < len; i++) {
            frame.data[i] = data[i];
        }

        while (nbytes <= 0 && nbTests < 10)
        {
            ozcore_can_socket_access_.lock();

            nbytes = write(ozcore_can_socket_desc_, &frame, sizeof(struct can_frame));

            ozcore_can_socket_access_.unlock();

            nbTests++;
        }

        if (nbytes <= 0) {
            std::cout << "Can write error." << std::endl;
        }
    }
    catch ( std::exception e ) {
        std::cout<<"Exception connect_can catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

// Read can packets from OzCore : Actuator, IHM, teleco setup

void Bridge::ozcore_read_can_thread( )
{
    try{

        using namespace std::chrono_literals;

        int naio01_ozcore_can_server_port = 5559;

        ssize_t bytesRead;

        struct can_frame frame;

        memset( &frame, 0, sizeof( frame ) );

        ozcore_can_server_socket_desc_ = DriverSocket::openSocketServer( naio01_ozcore_can_server_port );

        ozcore_read_can_thread_started_ = true;

        ozcore_can_socket_connected_ = false;

        while( !stop_main_thread_asked_)
        {
            if ( !ozcore_can_socket_connected_ and ozcore_can_server_socket_desc_ > 0 )
            {
                ozcore_can_socket_desc_ = DriverSocket::waitConnectTimer( ozcore_can_server_socket_desc_, stop_main_thread_asked_);

                std::this_thread::sleep_for( 50ms );

                if ( ozcore_can_socket_desc_ > 0 )
                {
                    ozcore_can_socket_connected_ = true;

                    ROS_ERROR( "OzCore Can Socket Connected" );
                }
            }
            else if( ozcore_can_socket_connected_ )
            {
                bytesRead = read( ozcore_can_socket_desc_, &frame, sizeof( frame ));

                if(bytesRead < 0)
                {
                    if(errno == 32){
                        disconnection_can();
                    }
                }
                else if (bytesRead == 0){
                    if( errno == 11 or errno == 32 ){
                        disconnection_can();
                    }
                }
                else if ( bytesRead > 0 )
                {

                    if( ( ( frame.can_id ) >> 7 ) == CAN_ID_IHM )
                    {

                        if( ( ( frame.can_id ) % 16 ) == CAN_IHM_LCD )
                        {
                            uint8_t car_position = frame.data[ 1 ];
                            char car = static_cast<char>( frame.data[ 2 ] );

                            if( car_position < 16 )
                            {
                                com_ozcore_ihm_line_top_[ car_position ] = car;
                            }
                            else
                            {
                                com_ozcore_ihm_line_bottom_[ car_position - 40 ] = car;
                            }
                        }
                    }
                    else if( ( ( frame.can_id ) >> 7 ) == CAN_ID_VER )
                    {

                        if( ( ( frame.can_id ) % 16 ) == CAN_VER_CONS )
                        {

                            asked_tool_position_access_.lock();
                            asked_tool_position_ = frame.data[ 0 ];
                            asked_tool_position_access_.unlock();

                            ApiMoveActuatorPacketPtr api_move_actuator_packet = std::make_shared<ApiMoveActuatorPacket>( frame.data[ 0 ] );

                            packet_list_to_send_access_.lock();
                            packet_list_to_send_.push_back( api_move_actuator_packet );
                            packet_list_to_send_access_.unlock();
                        }
                        else if( ( ( frame.can_id ) % 16 ) == CAN_VER_POS )
                        {
                            uint8_t data[1];

                            tool_position_access_.lock();
                            data[ 0 ] = tool_position_;
                            tool_position_access_.unlock();

                            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_VER, ComSimuCanMessageType::CAN_VER_POS, data, 1 );
                        }
                    }
                    else if( ( ( frame.can_id ) >> 7 ) == CAN_ID_TELECO )
                    {
                        if( ( ( frame.can_id ) % 16 ) == CAN_TELECO_NUM_VERSION )
                        {
                            uint8_t remote_data[ 8 ];

                            std::cout << "setting teleco act : " << static_cast<int>( frame.data[ 6 ] ) << " self_id : " << static_cast<int>(  frame.data[ 7 ] ) << std::endl;

                            com_ozcore_remote_status_.teleco_self_id_6 = frame.data[ 7 ];

                            remote_data[ 0 ] = 0x10;
                            remote_data[ 1 ] = 0x08;
                            remote_data[ 2 ] = 0x00;
                            remote_data[ 3 ] = 0x00;
                            remote_data[ 4 ] = 0x00;
                            remote_data[ 5 ] = 0x00;
                            remote_data[ 6 ] = com_ozcore_remote_status_.teleco_self_id_6;
                            remote_data[ 7 ] = com_ozcore_remote_status_.teleco_act_7;

                            if( com_ozcore_remote_status_.teleco_act_7 < 10 )
                            {
                                remote_data[ 2 ] = ( 4 + 128 );
                            }
                            else if( com_ozcore_remote_status_.teleco_act_7 > 10 )
                            {
                                remote_data[ 2 ] = ( 16 + 32 );
                            }

                            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_TELECO, ComSimuCanMessageType::CAN_TELECO_NUM_VERSION, remote_data, 8 );

                        }
                    }
                }

            }

            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );

        }
        close(ozcore_can_server_socket_desc_);

        disconnection_can();

        ozcore_read_can_thread_started_ = false;

    }
    catch ( std::exception e ) {
        std::cout<<"Exception read can catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

// Send teleco orders to OzCore

void Bridge::ozcore_remote_thread( )
{
    try {

        while (!stop_main_thread_asked_) {

            // Send teleco keys

            uint8_t remote_data[ 8 ];

            uint8_t directional_cross = 0x00;
            uint8_t buttons1 = 0x00;

            if( com_ozcore_remote_status_.pad_up )
            {
                directional_cross = ( directional_cross | ( 0x01 << 3 ) );
            }

            if( com_ozcore_remote_status_.pad_left )
            {
                directional_cross = ( directional_cross | ( 0x01 << 4 ) );
            }

            if( com_ozcore_remote_status_.pad_right )
            {
                directional_cross = ( directional_cross | ( 0x01 << 5 ) );
            }

            if( com_ozcore_remote_status_.pad_down )
            {
                directional_cross = ( directional_cross | ( 0x01 << 6 ) );
            }

            if( com_ozcore_remote_status_.secu_left )
            {
                buttons1 = ( buttons1 | ( 0x01 << 0 ) );
            }

            if( com_ozcore_remote_status_.secu_right )
            {
                buttons1 = ( buttons1 | ( 0x01 << 1 ) );
            }

            if( com_ozcore_remote_status_.arr_left )
            {
                buttons1 = ( buttons1 | ( 0x01 << 2 ) );
            }

            if( com_ozcore_remote_status_.arr_right )
            {
                buttons1 = ( buttons1 | ( 0x01 << 3 ) );
            }

            if( com_ozcore_remote_status_.tool_down )
            {
                buttons1 = ( buttons1 | ( 0x01 << 6 ) );
            }

            if( com_ozcore_remote_status_.tool_up )
            {
                buttons1 = ( buttons1 | ( 0x01 << 4 ) );
            }

            remote_data[ 0 ] = com_ozcore_remote_status_.analog_x;
            remote_data[ 1 ] = com_ozcore_remote_status_.analog_y;

            remote_data[ 2 ] = buttons1;
            remote_data[ 3 ] = directional_cross;

            remote_data[ 4 ] = 0x00;
            remote_data[ 5 ] = 0x00;

            remote_data[ 6 ] = com_ozcore_remote_status_.teleco_self_id_6;
            remote_data[ 7 ] = com_ozcore_remote_status_.teleco_act_7;

            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_TELECO, ComSimuCanMessageType::CAN_TELECO_KEYS, remote_data, 8 );

            std::this_thread::sleep_for(std::chrono::milliseconds(COM_OZCORE_REMOTE_SEND_RATE_MS / 2));


            // Send keypad can packet

            uint8_t keypad_data[ 1 ];

            uint8_t buttons = 0;

            if( com_ozcore_ihm_button_status_.cancel )
            {
                buttons = ( buttons | ( 0x01 << 0 ) );
            }
            if( com_ozcore_ihm_button_status_.validate )
            {
                buttons = ( buttons | ( 0x01 << 1 ) );
            }
            if( com_ozcore_ihm_button_status_.plus )
            {
                buttons = ( buttons | ( 0x01 << 2 ) );
            }
            if( com_ozcore_ihm_button_status_.minus )
            {
                buttons = ( buttons | ( 0x01 << 3 ) );
            }
            if( com_ozcore_ihm_button_status_.right )
            {
                buttons = ( buttons | ( 0x01 << 4 ) );
            }
            if( com_ozcore_ihm_button_status_.left )
            {
                buttons = ( buttons | ( 0x01 << 5 ) );
            }
            keypad_data[ 0 ] = buttons;

            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_IHM, ComSimuCanMessageType::CAN_IHM_BUT, keypad_data, 1 );

            std::this_thread::sleep_for(std::chrono::milliseconds(COM_OZCORE_REMOTE_SEND_RATE_MS / 2));
        }
    }
    catch ( std::exception e ) {
        std::cout<<"Exception remote_thread catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

void Bridge::disconnection_serial()
{
    close( ozcore_serial_socket_desc_ );

    ozcore_serial_connected_ = false;

    ROS_ERROR("OzCore Serial Socket Disconnected");
}

// ##################################################################################################

void Bridge::disconnection_can()
{
    close( ozcore_can_socket_desc_ );

    ozcore_can_socket_connected_ = false;

    ROS_ERROR("OzCore can Socket Disconnected");
}

// ##################################################################################################

int64_t Bridge::get_now_ms( )
{
    milliseconds now_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
    int64_t now = static_cast<int64_t>( now_ms.count() );

    return  now;
}

// ##################################################################################################

// Sends GPS packets to OzCore

void Bridge::gps_manager_thread_function( )
{
    try{

        while(!stop_main_thread_asked_ )
        {
            ha_gps_packet_ptr_access_.lock();
            HaGpsPacketPtr ha_gps_packet_ptr = ha_gps_packet_ptr_;
            HaGpsPacketPtr previous_ha_gps_packet_ptr = previous_ha_gps_packet_ptr_;
            ha_gps_packet_ptr_access_.unlock();

            if( (ha_gps_packet_ptr == nullptr) or (previous_ha_gps_packet_ptr == nullptr))
            {
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            }
            else
            {
                double track_orientation = 0.0;

                // compute speed and track orientation
                track_orientation = get_north_bearing( previous_ha_gps_packet_ptr->lat, previous_ha_gps_packet_ptr->lon, ha_gps_packet_ptr->lat, ha_gps_packet_ptr->lon );

//                #####################  RMC ####################

                std::time_t rawtime;
                std::tm* timeinfo;

                char hhmmss[ 80 ];
                char ddmmyy[ 80 ];
                char to[ 80 ];
                char gs[ 80 ];

                std::time( &rawtime );
                timeinfo = std::localtime( &rawtime );

                sprintf( to, "%03.1f", track_orientation );
                sprintf( gs, "%03.1f", ha_gps_packet_ptr_->groundSpeed );

                std::strftime( hhmmss, 80, "%H%M%S", timeinfo );
                std::strftime( ddmmyy, 80, "%d%m%y", timeinfo );

                GeoAngle ns = GeoAngle::from_double( ha_gps_packet_ptr->lat );
                GeoAngle we = GeoAngle::from_double( ha_gps_packet_ptr->lon );

                std::string gprmc =   string( "$GPRMC" ) + string( "," )
                                      + string( hhmmss ) + string( "," )
                                      + string( "A" ) + string( "," )
                                      + ns.to_string( true ) + string( "," )
                                      + we.to_string( false ) + string( "," )
                                      + string( gs ) + string( "," )
                                      + string( to ) + string( "," )
                                      + string( ddmmyy ) + string( "," )
                                      + string( "#" ) + string( "," )
                                      + string( "*" );

                //  #####################  VTG ####################

                std::string gpvtg =   string( "$GPVTG" ) + string( "," )
                                      + string( to ) + string( ",T," )
                                      + string( to ) + string( ",M," )
                                      + string( gs ) + string( ",N," )
                                      + string( gs ) + string( ",K," )
                                      + string( "*" );


                //  #####################  GGA ####################

                char quality[ 80 ];
                char nos[ 80 ];
                char alt[ 80 ];

                sprintf( quality, "%d", static_cast<int>( ha_gps_packet_ptr_->quality ) );
                sprintf( nos, "%02d", static_cast<int>( ha_gps_packet_ptr_->satUsed ) );
                sprintf( alt, "%03.1f", ha_gps_packet_ptr_->alt );

                std::string gpgga =   string( "$GPGGA" ) + string( "," )
                                      + string( "#" ) + string( "," )
                                      + string( "#" ) + string( "," )
                                      + string( "#" ) + string( "," )
                                      + string( "#" ) + string( "," )
                                      + string( "#" ) + string( "," )
                                      + string( quality ) + string( "," )
                                      + string( nos ) + string( "," )
                                      + string( "0.9" ) + string( "," )
                                      + string( alt ) + string( ",M," )
                                      + string( "*" );

                // ###############################
                // send gprmc
                for( int i = 0 ; i < gprmc.size() ; i++ )
                {
                    uint8_t data[ 1 ];

                    data[ 0 ] = static_cast<uint8_t>( gprmc.at( i ) );

                    ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_GPS, ComSimuCanMessageType::CAN_GPS_DATA, data, 1 );

                    usleep( 200 );
                }

                uint8_t end_data[ 1 ];
                end_data[ 0 ] = 10;

                ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_GPS, ComSimuCanMessageType::CAN_GPS_DATA, end_data, 1 );

                usleep( 200 );

                // ###############################
                // send gpvtg
                for( int i = 0 ; i < gpvtg.size() ; i++ )
                {
                    uint8_t data[ 1 ];

                    data[ 0 ] = static_cast<uint8_t>( gpvtg.at( i ) );

                    ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_GPS, ComSimuCanMessageType::CAN_GPS_DATA, data, 1 );

                    usleep( 200 );
                }

                ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_GPS, ComSimuCanMessageType::CAN_GPS_DATA, end_data, 1 );

                usleep( 200 );


                // ###############################
                // send gpgga
                for( int i = 0 ; i < gpgga.size() ; i++ )
                {
                    uint8_t data[ 1 ];

                    data[ 0 ] = static_cast<uint8_t>( gpgga.at( i ) );

                    ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_GPS, ComSimuCanMessageType::CAN_GPS_DATA, data, 1 );

                    usleep( 200 );
                }

                ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_GPS, ComSimuCanMessageType::CAN_GPS_DATA, end_data, 1 );

                usleep( 20 );
            }
        }
    }
    catch ( std::exception e ) {
        std::cout<<"Exception gps_manager catch : "<< e.what() << std::endl;
    }
}

// ##################################################################################################

double Bridge::get_north_bearing( double lat1, double lon1, double lat2, double lon2 )
{
    double startLat = lat1 * M_PI / 180.0;
    double startLon = lon1  * M_PI / 180.0;

    double endLat = lat2 * M_PI / 180.0;
    double endLon = lon2  * M_PI / 180.0;

    double dLong = endLon - startLon;

    double dPhi = std::log( std::tan( endLat / 2.0 + M_PI / 4.0 ) / std::tan( startLat / 2.0 + M_PI / 4.0 ) );

    if( std::abs( dLong ) > M_PI )
    {
        if( dLong > 0.0 )
        {
            dLong = -( 2.0 * M_PI - dLong );
        }
        else
        {
            dLong = ( 2.0 * M_PI - dLong );
        }
    }

    double brng = fmod( ( std::atan2( dLong, dPhi ) / M_PI * 180.0  ) + 360.0 , 360.0 );

    return brng;
}