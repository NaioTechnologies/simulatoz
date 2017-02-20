
#include "DriverSocket.hpp"

#include "Camera.h"

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Camera::Camera( uint16_t server_port)
        : stop_{false}
        , connect_thread_started_ {false}
        , connect_thread_ {}
        , socket_connected_ {false}
        , server_port_ {server_port}
        , server_socket_desc_ {-1}
        , socket_desc_ {-1}
        , image_left_sub_{ }
        , image_right_sub_{ }
        , sync{ image_left_sub_, image_right_sub_, 10 }
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

//*****************************************  --  SUBSCRIBE  --  *************************************************************

void Camera::subscribe( ros::NodeHandle& node )
{
    // subscribe to camera topic
    image_left_sub_.subscribe( node, "/oz440/camera/left/image_raw", 1 );
    image_right_sub_.subscribe( node, "/oz440/camera/right/image_raw", 1 );
    sync.registerCallback( boost::bind( &Camera::callback_camera, this, _1, _2 ) );
}

//*****************************************  --  ASK STOP  --  *********************************************************

void Camera::cleanup()
{
    stop_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECT  --  **********************************************************

void Camera::connect(){

    connect_thread_started_ = true;

    server_socket_desc_ = DriverSocket::openSocketServer( server_port_ );

    while ( !stop_ )
    {
        if( !socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_ );

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

void Camera::send_image( std::array< uint8_t, buffer_size_ > image ){

    using namespace std::chrono_literals;

    int total_written_bytes = 0;
    ssize_t write_size = 0;

    while ( total_written_bytes < buffer_size_ and socket_connected_)
    {
        write_size = send( socket_desc_, image.data() + total_written_bytes, buffer_size_ - total_written_bytes, 0 );

        if ( write_size <= 0 )
        {
            if ( errno == 32 or errno == 104 )
            {
                disconnect();
            }
        }
        else
        {
            total_written_bytes = total_written_bytes + static_cast<int>( write_size );
        }
    }

    if ( total_written_bytes == buffer_size_ )
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

// *********************************************************************************************************************

void
Camera::callback_camera( const sensor_msgs::Image::ConstPtr& image_left,
                         const sensor_msgs::Image::ConstPtr& image_right )
{
    if( socket_connected_ )
    {
        std::array< uint8_t, buffer_size_ > image;

        std::memcpy( image.data(), &image_left->data[0], buffer_size_ / 2 );
        std::memcpy( image.data() + buffer_size_ / 2, &image_right->data[0], buffer_size_ / 2 );

        send_image( image );
    }
}