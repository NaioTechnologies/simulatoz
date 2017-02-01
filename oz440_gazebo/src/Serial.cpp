
#include "../include/DriverSocket.hpp"

#include "../include/Serial.h"

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Serial::Serial(int server_port)
        : stop_asked_{ false }
        , connect_thread_started_ { false }
        , connect_thread_ { }
        , read_thread_started_ { false }
        , read_thread_ { }
        , socket_connected_ { false }
        , server_port_ {server_port}
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ { }
        , packet_ptr_ { nullptr }
        , packet_access_ { }
{
    init();
}

Serial::~Serial()
{
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  INIT  --  *************************************************************

void Serial::init()
{
    connect_thread_ = std::thread( &Serial::connect, this );
    connect_thread_.detach();

    read_thread_ = std::thread( &Serial::read_thread, this );
    read_thread_.detach();
}

//*****************************************  --  SET PACKET  --  *******************************************************

HaMotorsPacketPtr Serial::get_packet()
{
    packet_access_.lock();
    HaMotorsPacketPtr packet_ptr = packet_ptr_;
    packet_ptr_ = nullptr;
    packet_access_.unlock();

    return packet_ptr;
}

//*****************************************  --  ASK STOP  --  *********************************************************

void Serial::ask_stop()
{
    stop_asked_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECTED?  --  *******************************************************

bool Serial::connected(){
    return socket_connected_;
}

//*****************************************  --  CONNECT  --  **********************************************************

void Serial::connect(){

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

                ROS_ERROR( "OzCore Serial Socket Connected" );
            }
        }
        else{
            std::this_thread::sleep_for(500ms);
        }
    }

    connect_thread_started_ = false;
}

//*****************************************  --  READ THREAD  --  ******************************************************

void Serial::read_thread(){

    unsigned char b[200];

    int motorNumber = 0;
    int posInEntete = 0;
    char motors[ 3 ] = { 0 };

    read_thread_started_ = true;

    while ( !stop_asked_ )
    {
        if (socket_connected_)
        {
            socket_access_.lock();
            ssize_t size = read( socket_desc_, b, 1 );
            socket_access_.unlock();

            if (size > 0)
            {

                if ( posInEntete == 2 )
                {
                    motors[ motorNumber ] = ( ( ( char ) b[ 0 ] ) * 2 ) - 128;

                    if ( motorNumber == 2 )
                    {
                        HaMotorsPacketPtr packet_ptr = std::make_shared<HaMotorsPacket>( motors[ 2 ], motors[ 1 ] );

                        packet_access_.lock();
                        packet_ptr_ = packet_ptr ;
                        packet_access_.unlock();
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
            else
            {
                if( errno == 32 or errno == 104 or ( size == 0 and errno == 11 )){
                    disconnect();
                }
            }
        }

        std::this_thread::sleep_for(5ms);
    }

    read_thread_started_ = false;
}

//*****************************************  --  DISCONNECT  --  *******************************************************

void Serial::disconnect(){

    close( socket_desc_ );

    socket_connected_ = false;

    ROS_ERROR("OzCore Serial Socket Disconnected");
}