
#include "../include/DriverSocket.hpp"

#include "../include/Lidar.h"

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Lidar::Lidar(int server_port)
        : stop_asked_{ false }
        , connect_thread_started_ { false }
        , connect_thread_ { }
        , read_thread_started_ { false }
        , read_thread_ { }
        , socket_connected_ { false }
        , server_port_ {server_port}
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ {}
        , packet_ptr_ { nullptr }
        , packet_access_ {}
        , nbMesures_ { 1 }
        , nbTelegrammes_ { 1 }
{
    init();
}

Lidar::~Lidar()
{
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  INIT  --  *************************************************************

void Lidar::init()
{
    connect_thread_ = std::thread( &Lidar::connect, this );
    connect_thread_.detach();

    read_thread_ = std::thread( &Lidar::read_thread, this );
    read_thread_.detach();
}

//*****************************************  --  SET PACKET  --  *******************************************************

void Lidar::set_packet( HaLidarPacketPtr packet_ptr )
{
    packet_access_.lock();
    packet_ptr_ = packet_ptr;
    packet_access_.unlock();

    send_packet();
}

//*****************************************  --  ASK STOP  --  *********************************************************

void Lidar::ask_stop()
{
    stop_asked_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECTED?  --  *******************************************************

bool Lidar::connected(){
    return socket_connected_;
}

//*****************************************  --  CONNECT  --  **********************************************************

void Lidar::connect(){

    connect_thread_started_ = true;

    server_socket_desc_ = DriverSocket::openSocketServer( (uint16_t) server_port_ );

    while( !stop_asked_ )
    {
        if( !socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_asked_ );

            if ( socket_desc_ > 0 )
            {
                socket_connected_ = true;

                ROS_ERROR( "OzCore Lidar Socket Connected" );
            }
        }
        else{
            std::this_thread::sleep_for(500ms);
        }
    }

    connect_thread_started_ = false;
}

//*****************************************  --  READ THREAD  --  ******************************************************

void Lidar::read_thread(){

    char received_buffer[4096];

    read_thread_started_ = true;

    while ( !stop_asked_ )
    {
        if (socket_connected_)
        {
            memset( received_buffer, '\0', 1000 );

            socket_access_.lock();
            ssize_t size = read( socket_desc_, received_buffer, 4096 );
            socket_access_.unlock();

            if (size > 0)
            {
                received_buffer[ size ] = '\0';

                if (strncmp("\x02sRN LMDscandata 1\x03", (char *) received_buffer, strlen("\x02sRN LMDscandata 1\x03")) == 0)
                {
//                    send_packet();
                }
            }
            else
            {
                if( errno == 32 or errno == 104 or ( size == 0 and errno == 11 )){
                    disconnect();
                }
            }

            std::this_thread::sleep_for(1ms);
        }
        else
        {
            std::this_thread::sleep_for(100ms);
        }
    }
    read_thread_started_ = false;

}

//*****************************************  --  DISCONNECT  --  *******************************************************

void Lidar::disconnect(){

    close( socket_desc_ );

    socket_connected_ = false;

    ROS_ERROR("OzCore Lidar Socket Disconnected");
}

//*****************************************  --  SEND PACKET  --  ******************************************************

void Lidar::send_packet(){

    char trame[10000];

    int lidar[271];
    int albedo[271];

    struct timespec timeInit;
    clock_gettime(CLOCK_MONOTONIC_RAW, &timeInit);

    if (socket_connected_) {

        packet_access_.lock();

        if (packet_ptr_ != nullptr) {
            for (int i = 0; i < 271; i++) {
                lidar[i] = packet_ptr_->distance[i];
                albedo[i] = packet_ptr_->albedo[i];
            }
        }

        packet_access_.unlock();

        nbMesures_++;
        nbTelegrammes_++;

        createTrame(lidar, albedo, trame, nbMesures_, nbTelegrammes_, timeInit);

        socket_access_.lock();
        ssize_t write_size = write(socket_desc_, trame, strlen(trame));
        socket_access_.unlock();

        if (write_size != strlen(trame)) {
            ROS_ERROR("Error sending Lidar trame");
        } else {
            ROS_INFO("Lidar packet send");
        }
    }
}