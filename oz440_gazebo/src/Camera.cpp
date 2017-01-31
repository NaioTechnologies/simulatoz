
#include "../include/DriverSocket.hpp"

#include "../include/Camera.h"

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Camera::Camera(int server_port)
        : stop_asked_{false}
        , connect_thread_started_ {false}
        , connect_thread_ {}
        , socket_connected_ {false}
        , server_port_ {server_port}
        , server_socket_desc_ {-1}
        , socket_desc_ {-1}
        , image_access_ {}
        , image_ {}
{
    init();
}

Camera::~Camera()
{
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  INIT  --  *************************************************************

void Camera::init()
{
    connect_thread_ = std::thread( &Camera::connect, this );
    connect_thread_.detach();
}

//*****************************************  --  SET_IMAGE  --  ********************************************************

void Camera::set_image(std::array < uint8_t, 721920 > image)
{
    image_access_.lock();
    image_ = image;
    image_access_.unlock();

    send_image();
}

//*****************************************  --  ASK STOP  --  *********************************************************

void Camera::ask_stop()
{
    stop_asked_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECTED?  --  *******************************************************

bool Camera::connected(){
    return socket_connected_;
}

//*****************************************  --  CONNECT  --  **********************************************************

void Camera::connect(){

    connect_thread_started_ = true;

    server_socket_desc_ = DriverSocket::openSocketServer( server_port_ );

    while ( !stop_asked_ )
    {
        if( !socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_asked_ );

            if ( socket_desc_ > 0 )
            {
                socket_connected_ = true;

                ROS_ERROR( "OzCore Image Socket Connected" );
            }
        }
        else
        {
            std::this_thread::sleep_for( 500ms );
        }
    }

    connect_thread_started_ = false;
}

//*****************************************  --  SEND_IMAGE  --  *******************************************************

void Camera::send_image(){

    using namespace std::chrono_literals;

    int total_written_bytes = 0;
    ssize_t write_size = 0;
    int nb_tries = 0;
    int max_tries = 20;

    image_access_.lock();

    while ( total_written_bytes < 721920 and nb_tries < max_tries and !stop_asked_ and socket_connected_) {

        write_size = send( socket_desc_, image_.data() + total_written_bytes, 721920 - total_written_bytes, 0 );

        if ( write_size <= 0 )
        {
            if ( errno == 32 or errno == 104 )
            {
                disconnect();
            }
            nb_tries++;
        }
        else
        {
            total_written_bytes = total_written_bytes + static_cast<int>( write_size );
            nb_tries = 0;
        }
        std::this_thread::sleep_for( 5ms );
    }

    image_access_.unlock();

    if ( nb_tries >= max_tries )
    {
        ROS_ERROR("send packet failed, too many tries");
    }
    else if ( total_written_bytes == 721920 )
    {
        ROS_INFO("Camera packet send");
    }
}

//*****************************************  --  DISCONNECT  --  *******************************************************

void Camera::disconnect(){

    close( socket_desc_ );

    socket_connected_ = false;

    ROS_ERROR("OzCore Image Socket Disconnected");
}