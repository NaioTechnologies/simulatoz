
#include "DriverSocket.hpp"

#include "Serial.h"

#include <std_srvs/Empty.h>

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Serial::Serial(uint16_t server_port)
        : stop_{ false }
        , connect_thread_ { }
        , socket_connected_ { false }
        , server_port_ {server_port}
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ { }
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

//*****************************************  --  ADVERTISE  --  *************************************************************

void Serial::advertise(ros::NodeHandle& node)
{
    velocity_pub_ = node.advertise< geometry_msgs::Vector3 >( "/oz440/cmd_vel", 10 );
}

//*****************************************  --  ASK STOP  --  *********************************************************

void Serial::cleanup()
{
    stop_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECT  --  **********************************************************

void Serial::connect(){

    server_socket_desc_ = DriverSocket::openSocketServer( server_port_ );

    while( !stop_ )
    {
        if( !socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_ );

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
}

//*****************************************  --  READ THREAD  --  ******************************************************

void Serial::read_thread(){

    unsigned char b[200];

    int motorNumber = 0;
    int posInEntete = 0;
    char motors[ 3 ] = { 0 };

    while ( !stop_ )
    {
        if (socket_connected_)
        {
            socket_access_.lock();
            ssize_t size = read( socket_desc_, b, 1 );
            socket_access_.unlock();

            if (size > 0)
            {
                if (posInEntete == 2)
                {
                    motors[motorNumber] = (((char) b[0]) * 2) - 128;

                    if (motorNumber == 2) {
                        geometry_msgs::Vector3 command;

                        command.x = ((motors[2] / 127.0) * 3.4);
                        command.y = ((motors[1] / 127.0) * 3.4);

                        velocity_pub_.publish(command);
                    }

                    posInEntete = 0;
                }

                if (posInEntete == 1)
                {
                    if (b[0] == 6)
                    {
                        motorNumber = 1;
                        posInEntete = 2;
                    }
                    else if (b[0] == 7)
                    {
                        motorNumber = 2;
                        posInEntete = 2;
                    }
                    else
                    {
                        posInEntete = 0;
                    }
                }

                if ( (posInEntete == 0) && (b[0] == 128) ) {
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
}

//*****************************************  --  DISCONNECT  --  *******************************************************

void Serial::disconnect(){

    close( socket_desc_ );

    socket_connected_ = false;

    ROS_ERROR("OzCore Serial Socket Disconnected");

    std_srvs::Empty message;
    ros::service::call("gazebo/reset_world", message);
}
