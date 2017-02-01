
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


// com_ozcore
#include "../include/DriverSerial.hpp"


using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

//static pid_t gettid( void )
//{
//    return syscall( __NR_gettid );
//}

// #####################################################################################################################
// ---------------------------------------------------------------------------------------------------------------------
// #####################################################################################################################

Bridge::Bridge( ) :
        stop_main_thread_asked_{ false },
        packet_list_to_send_{ },
        control_type_{ ControlType::CONTROL_TYPE_MANUAL },
        last_motor_time_{ 0L },
        ozcore_serial_connected_{false},
        bridge_connected_{false}
{
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

void Bridge::init()
{
    try
    {
        ozcore_read_serial_thread_ = std::thread(&Bridge::ozcore_read_serial_thread, this);

//        ozcore_remote_thread_ = std::thread(&Bridge::ozcore_remote_thread, this);

        bridge_connected_ = true;

        ozcore_read_serial_thread_.join();
//        ozcore_remote_thread_.join();

        ROS_ERROR("End of bridge thread");
    }
    catch ( std::exception e ) {
        std::cout<<"Exception init catch : "<< e.what() << std::endl;
    }
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

//######################################################################################################################

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

//// Send teleco orders to OzCore
//
//void Bridge::ozcore_remote_thread( )
//{
//    try {
//
//        while (!stop_main_thread_asked_) {
//
//            // Send teleco keys
//
//            uint8_t remote_data[ 8 ];
//
//            uint8_t directional_cross = 0x00;
//            uint8_t buttons1 = 0x00;
//
//            if( com_ozcore_remote_status_.pad_up )
//            {
//                directional_cross = ( directional_cross | ( 0x01 << 3 ) );
//            }
//
//            if( com_ozcore_remote_status_.pad_left )
//            {
//                directional_cross = ( directional_cross | ( 0x01 << 4 ) );
//            }
//
//            if( com_ozcore_remote_status_.pad_right )
//            {
//                directional_cross = ( directional_cross | ( 0x01 << 5 ) );
//            }
//
//            if( com_ozcore_remote_status_.pad_down )
//            {
//                directional_cross = ( directional_cross | ( 0x01 << 6 ) );
//            }
//
//            if( com_ozcore_remote_status_.secu_left )
//            {
//                buttons1 = ( buttons1 | ( 0x01 << 0 ) );
//            }
//
//            if( com_ozcore_remote_status_.secu_right )
//            {
//                buttons1 = ( buttons1 | ( 0x01 << 1 ) );
//            }
//
//            if( com_ozcore_remote_status_.arr_left )
//            {
//                buttons1 = ( buttons1 | ( 0x01 << 2 ) );
//            }
//
//            if( com_ozcore_remote_status_.arr_right )
//            {
//                buttons1 = ( buttons1 | ( 0x01 << 3 ) );
//            }
//
//            if( com_ozcore_remote_status_.tool_down )
//            {
//                buttons1 = ( buttons1 | ( 0x01 << 6 ) );
//            }
//
//            if( com_ozcore_remote_status_.tool_up )
//            {
//                buttons1 = ( buttons1 | ( 0x01 << 4 ) );
//            }
//
//            remote_data[ 0 ] = com_ozcore_remote_status_.analog_x;
//            remote_data[ 1 ] = com_ozcore_remote_status_.analog_y;
//
//            remote_data[ 2 ] = buttons1;
//            remote_data[ 3 ] = directional_cross;
//
//            remote_data[ 4 ] = 0x00;
//            remote_data[ 5 ] = 0x00;
//
//            remote_data[ 6 ] = com_ozcore_remote_status_.teleco_self_id_6;
//            remote_data[ 7 ] = com_ozcore_remote_status_.teleco_act_7;
//
//            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_TELECO, ComSimuCanMessageType::CAN_TELECO_KEYS, remote_data, 8 );
//
//            std::this_thread::sleep_for(std::chrono::milliseconds(COM_OZCORE_REMOTE_SEND_RATE_MS / 2));
//
//
//            // Send keypad can packet
//
//            uint8_t keypad_data[ 1 ];
//
//            uint8_t buttons = 0;
//
//            if( com_ozcore_ihm_button_status_.cancel )
//            {
//                buttons = ( buttons | ( 0x01 << 0 ) );
//            }
//            if( com_ozcore_ihm_button_status_.validate )
//            {
//                buttons = ( buttons | ( 0x01 << 1 ) );
//            }
//            if( com_ozcore_ihm_button_status_.plus )
//            {
//                buttons = ( buttons | ( 0x01 << 2 ) );
//            }
//            if( com_ozcore_ihm_button_status_.minus )
//            {
//                buttons = ( buttons | ( 0x01 << 3 ) );
//            }
//            if( com_ozcore_ihm_button_status_.right )
//            {
//                buttons = ( buttons | ( 0x01 << 4 ) );
//            }
//            if( com_ozcore_ihm_button_status_.left )
//            {
//                buttons = ( buttons | ( 0x01 << 5 ) );
//            }
//            keypad_data[ 0 ] = buttons;
//
//            ozcore_send_can_packet( ComSimuCanMessageId::CAN_ID_IHM, ComSimuCanMessageType::CAN_IHM_BUT, keypad_data, 1 );
//
//            std::this_thread::sleep_for(std::chrono::milliseconds(COM_OZCORE_REMOTE_SEND_RATE_MS / 2));
//        }
//    }
//    catch ( std::exception e ) {
//        std::cout<<"Exception remote_thread catch : "<< e.what() << std::endl;
//    }
//}

// ##################################################################################################

void Bridge::disconnection_serial()
{
    close( ozcore_serial_socket_desc_ );

    ozcore_serial_connected_ = false;

    ROS_ERROR("OzCore Serial Socket Disconnected");
}