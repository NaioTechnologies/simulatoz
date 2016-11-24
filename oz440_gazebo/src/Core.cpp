#include "Core.hpp"
#include "DriverSocket.hpp"
#include <chrono>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <geometry_msgs/PointStamped.h>

#include "../include/oz440_socket/SocketException.h"

#include "oz440_api/HaLidarPacket.hpp"
#include "oz440_api/HaMotorsPacket.hpp"
#include "oz440_api/HaGpsPacket.hpp"
#include "oz440_api/HaGyroPacket.hpp"
#include "oz440_api/HaAcceleroPacket.hpp"
#include "oz440_api/ApiStereoCameraPacket.hpp"
#include "oz440_api/HaOdoPacket.hpp"
#include "oz440_api/ApiMoveActuatorPacket.hpp"

#include <vector>
#include <stdlib.h>

#define IMAGE_SIZE 752*480
#define HEADER_SIZE 15

using namespace std::chrono;

// *********************************************************************************************************************

int main( int argc, char **argv )
{
    Core core( argc, argv );

    core.run( );

    return 0;
}

// *********************************************************************************************************************

Core::Core( int argc, char **argv )
{
    ros::init( argc, argv, "core");

    ros::NodeHandle n;

    listener_ptr_ = std::make_shared< tf::TransformListener>( ros::Duration(10) );

    client_read_thread_started_ = false;

    client_socket_connected_ = false;

    image_thread_started_ = false;

    image_socket_connected_ = false;

    ozcore_image_thread_started_ = false;

    ozcore_image_socket_connected_ = false;

    image_packet_to_send_ = nullptr;

    last_image_ms_ = 0;

    last_ozcore_image_ms_ = 0;

    actuator_position_ = 0.0;

}

// *********************************************************************************************************************

Core::~Core( )
{
}

// *********************************************************************************************************************

void Core::run( )
{
    using namespace std::chrono_literals;

    //signal( SIGPIPE, SIG_IGN );

    ros::NodeHandle n;

    velocity_pub_ = n.advertise<geometry_msgs::Vector3>( "/oz440/cmd_vel", 10 );
    actuator_pub_ = n.advertise<geometry_msgs::Vector3>( "/oz440/cmd_act", 10 );

    //ros::Rate loop_rate(10);

    // subscribe to lidar topic
    ros::Subscriber lidar_sub = n.subscribe( "/oz440/laser/scan", 500000, &Core::send_lidar_packet_callback, this );

    // subscribe to actuator position
    ros::Subscriber actuator_position_sub = n.subscribe( "/oz440/joint_states", 500000, &Core::send_actuator_position_callback, this );

    // subscribe to imu topic
    ros::Subscriber imu_sub = n.subscribe("/oz440/imu/data", 50, &Core::send_imu_packet_callback, this);

    // subscribe to gps topic
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_fix_sub( n, "/oz440/navsat/fix", 5 );
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gps_vel_sub( n, "/oz440/navsat/vel", 5 );
    message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::Vector3Stamped> sync_gps( gps_fix_sub, gps_vel_sub, 10 );
    sync_gps.registerCallback( boost::bind( &Core::send_gps_packet_callback, this, _1, _2 ) );

    // subscribe to camera topic
    message_filters::Subscriber<sensor_msgs::Image> image_left_sub ( n, "/oz440/camera/left/image_raw", 1 );
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub ( n, "/oz440/camera/right/image_raw", 1 );
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_left_sub, image_right_sub, 10);
    sync.registerCallback ( boost::bind(&Core::send_camera_packet_callback, this, _1, _2) );

    // initialize server naio
    int naio01_server_port = 5555;

    // creates main thread
    client_read_thread_ = std::thread( &Core::client_read_thread_function, this );

    // creates image thread
    image_thread_ = std::thread( &Core::image_thread_function, this );

    // creates image thread
    image_read_thread_ = std::thread( &Core::image_read_thread_function, this );

    // creates ozcore_image thread
    ozcore_image_thread_ = std::thread( &Core::ozcore_image_thread_function, this );

    // creates ozcore_image thread
    ozcore_image_read_thread_ = std::thread( &Core::ozcore_image_read_thread_function, this );

    // create_odo_thread
    send_odo_thread_ = std::thread( &Core::send_odo_packet, this );

    server_socket_desc_ = DriverSocket::openSocketServer( naio01_server_port );

    client_socket_connected_ = false;

    while (ros :: ok())
    {
        if( not client_socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_access_.lock();

            client_socket_desc_ = DriverSocket::waitConnect(server_socket_desc_);

            socket_access_.unlock();

            if (client_socket_desc_ > 0)
            {
                client_socket_connected_ = true;

                ROS_INFO( "Connexion Socket");

                milliseconds now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                last_socket_activity_time_ = static_cast<int64_t>( now_ms.count());
            }
        }

        if( client_socket_connected_ )
        {
            milliseconds now_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
            int64_t now = static_cast<int64_t>( now_ms.count() );

            if( now - last_socket_activity_time_ > 1000 )
            {
                disconnected();
            }
        }

        packet_to_send_list_access_.lock();

        for( BaseNaio01PacketPtr packet : packet_to_send_list_ )
        {
            cl::BufferUPtr buffer = packet->encode();

            socket_access_.lock();

            int total_written_bytes = 0;
            int write_size = 0;

            while( total_written_bytes < buffer->size() and write_size >= 0 )
            {
                write_size = (int) write( client_socket_desc_, buffer->data() + total_written_bytes, buffer->size() - total_written_bytes);

                total_written_bytes = total_written_bytes + write_size;

            }

            socket_access_.unlock();
        }

        packet_to_send_list_.clear();

        packet_to_send_list_access_.unlock();

        std::this_thread::sleep_for(10ms);
    }

    close( server_socket_desc_ );
}

// *********************************************************************************************************************

void Core::image_disconnected()
{
    close( image_socket_desc_ );

    image_socket_connected_ = false;

    ROS_ERROR("Image Socket Disconnected");
}

// *********************************************************************************************************************

void Core::ozcore_image_disconnected()
{
    close( ozcore_image_socket_desc_ );

    ozcore_image_socket_connected_ = false;

    ROS_ERROR("OzCore Image Socket Disconnected");

}

// *********************************************************************************************************************

void Core::disconnected()
{
    close( client_socket_desc_ );

    client_socket_connected_ = false;

    packet_to_send_list_access_.lock();

    packet_to_send_list_.clear();

    packet_to_send_list_access_.unlock();

    system("rosservice call gazebo/reset_world");

}

// *********************************************************************************************************************

void Core::client_read_thread_function( )
{
    using namespace std::chrono_literals;

    uint8_t received_buffer[ 4096 ];
    bool packet_header_detected = false;
    bool at_least_one_packet_decoded = false;

    Naio01Codec naio_01_codec;
    std::vector< BaseNaio01PacketPtr > received_packet_list;

    client_read_thread_started_ = true;

    try
    {
        while ( ros::ok() )
        {

            if( client_socket_connected_ == true )
            {
                socket_access_.lock();

                ssize_t size  = read( client_socket_desc_, received_buffer, 4096 );

                socket_access_.unlock();

                if (size > 0)
                {
                    milliseconds now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                    last_socket_activity_time_ = static_cast<int64_t>( now_ms.count());

                    at_least_one_packet_decoded = naio_01_codec.decode(received_buffer, size, packet_header_detected);
                }

                if ( at_least_one_packet_decoded )
                {
                    // If successfull decoding emplace packets in the received buffer
                    for (uint i = 0; i < naio_01_codec.currentBasePacketList.size(); i++)
                    {
                        received_packet_list.emplace_back(naio_01_codec.currentBasePacketList[i]);
                    }

                    naio_01_codec.currentBasePacketList.clear();

                    for (uint i = 0; i < received_packet_list.size(); i++) // For every packet decoded
                    {
                        BaseNaio01PacketPtr basePacketPtr = received_packet_list.at(i);

                        if (std::dynamic_pointer_cast<HaMotorsPacket>(basePacketPtr))
                        {
                            //  When receiving a motor order
                            HaMotorsPacketPtr motorsPacketPtr = std::dynamic_pointer_cast<HaMotorsPacket>(basePacketPtr);

                            double rightspeed = static_cast<double>(motorsPacketPtr->right);
                            double leftspeed = static_cast<double>(motorsPacketPtr->left);

                            ROS_INFO("ApiMotorsPacket received, right: %f left : %f", rightspeed, leftspeed);

                            geometry_msgs::Vector3 command;

                            command.x = ((leftspeed / 127.0) * 3.4);
                            command.y = ((rightspeed / 127.0) * 3.4);

                            velocity_pub_.publish(command);
                        }

                        if (std::dynamic_pointer_cast<ApiMoveActuatorPacket>(basePacketPtr))
                        {
                            //  When receiving an actuator order
                            ApiMoveActuatorPacketPtr ActuatorPacketPtr = std::dynamic_pointer_cast<ApiMoveActuatorPacket>(basePacketPtr);

                            geometry_msgs::Vector3 command;

                            if ( ActuatorPacketPtr->position == 1 )
                            {
                                command.x = actuator_position_ +0.005;
                                ROS_ERROR("MONTE : %f", actuator_position_ +0.005);
//                                actuator_pub_.publish(command);

                            }
                            else if ( ActuatorPacketPtr->position == 2 )
                            {
                                command.x = actuator_position_ -0.005;
                                ROS_ERROR("DESCEND : %f", actuator_position_ -0.005);
//                                actuator_pub_.publish(command);
                            }
                            else
                            {
                                command.x = actuator_position_;
                            }

                            ROS_INFO("ApiMoveActuatorPacket received, position: %f ", command.x);

                            actuator_pub_.publish(command);
                        }
                    }
                    received_packet_list.clear();
                }
            }
            std::this_thread::sleep_for(1ms);

            ros::spinOnce();
        }
    }
    catch (SocketException& e )
    {
        ROS_INFO( "Client_read_thread_function exception was caught : %s", e.description().c_str() );
    }

    client_read_thread_started_ = false;
}

// *********************************************************************************************************************

void Core::image_read_thread_function( )
{
    using namespace std::chrono_literals;

    uint8_t received_buffer[ 4096 ];

    image_read_thread_started_ = true;

    try
    {
        while ( ros::ok() )
        {

            if( image_socket_connected_ == true )
            {

                image_socket_access_.lock();

                ssize_t size = read(image_socket_desc_, received_buffer, 4096);

                image_socket_access_.unlock();

                if (size > 0) {
                    milliseconds now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                    last_image_socket_activity_time_ = static_cast<int64_t>( now_ms.count());

                }
            }
            std::this_thread::sleep_for(250ms);
        }
    }
    catch (SocketException& e )
    {
        ROS_INFO( "Image_read_thread_function exception was caught : %s", e.description().c_str() );
    }

    image_read_thread_started_ = false;
}

//**********************************************************************************************************************

void Core::image_thread_function( )
{

    using namespace std::chrono_literals;
    int naio01_image_server_port = 5557;

//    uint8_t buffer_to_send[ 4000000 ];

    image_thread_started_ = true;

    image_server_socket_desc_ = DriverSocket::openSocketServer(naio01_image_server_port);

    image_socket_connected_ = false;

    int64_t last_image_sent_ms = 0;

    while (ros::ok())
    {
        if (not image_socket_connected_ and image_server_socket_desc_ > 0)
        {
            image_socket_access_.lock();

            image_socket_desc_ = DriverSocket::waitConnect(image_server_socket_desc_);

            image_socket_access_.unlock();

            if (image_socket_desc_ > 0) {
                image_socket_connected_ = true;

                ROS_ERROR("Connexion Image Socket");

                milliseconds image_now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                last_image_socket_activity_time_ = static_cast<int64_t>( image_now_ms.count());
            }
        }

        if (image_socket_connected_)
        {
            milliseconds image_now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
            int64_t now = static_cast<int64_t>( image_now_ms.count());

            if (now - last_image_socket_activity_time_ > 5000)
            {
                image_disconnected();
            }
            else
            {
                bool new_image_received = false;

                image_packet_to_send_access_.lock();

                cl::BufferUPtr buffer = image_packet_to_send_->encode();

                if( last_image_sent_ms != last_image_ms_ )
                {
                    new_image_received = true;

                    last_image_sent_ms = last_image_ms_;
                }

                image_packet_to_send_access_.unlock();

                if ( new_image_received == true )
                {

                    int total_written_bytes = 0;
                    ssize_t write_size = 0;

                    int nb_tries = 0;
                    int max_tries = 500;

                    while ( total_written_bytes < buffer->size() and nb_tries < max_tries )
                    {
                        write_size = send( image_socket_desc_, buffer->data() + total_written_bytes, buffer->size() - total_written_bytes, 0 );
                        std::this_thread::sleep_for( 5ms );

                        if ( write_size < 0 )
                        {
                            nb_tries++;
                            std::this_thread::sleep_for( 10ms );
                        }
                        else
                        {
                            total_written_bytes = total_written_bytes + static_cast<int>( write_size );
                            nb_tries = 0;
                        }
                    }

                    if (nb_tries >= max_tries)
                    {
                        ROS_ERROR("send packet failed, too many tries");
                    }
                    else
                    {
                        ROS_INFO("Camera packet send to 5557");
                    }
                }
            }
        }
        std::this_thread::sleep_for( 10ms);
    }

    close(image_server_socket_desc_);

    image_thread_started_ = false;

}

// *********************************************************************************************************************

void Core::ozcore_image_read_thread_function()
{
    using namespace std::chrono_literals;

    uint8_t received_buffer[ 4096 ];

    ozcore_image_read_thread_started_ = true;

    try
    {
        while ( ros::ok() )
        {

            if( ozcore_image_socket_connected_ == true )
            {
                ozcore_image_socket_access_.lock();

                ssize_t size = read( ozcore_image_socket_desc_, received_buffer, 4096 );

                ozcore_image_socket_access_.unlock();

                if (size > 0)
                {
                    milliseconds now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                    last_ozcore_image_socket_activity_time_ = static_cast<int64_t>( now_ms.count());
                }
            }
            std::this_thread::sleep_for(250ms);
        }
    }
    catch (SocketException& e )
    {
        ROS_INFO( "Image_read_thread_function exception was caught : %s", e.description().c_str() );
    }

    ozcore_image_read_thread_started_ = false;
}

//**********************************************************************************************************************

void Core::ozcore_image_thread_function( )
{
    using namespace std::chrono_literals;
    int naio01_ozcore_image_server_port = 5558;

    ozcore_image_thread_started_ = true;

    ozcore_image_server_socket_desc_ = DriverSocket::openSocketServer( naio01_ozcore_image_server_port );

    ozcore_image_socket_connected_ = false;

    int64_t last_ozcore_image_sent_ms = 0;

    uint8_t image_buffer_to_send[ 721920 ];

    while ( ros::ok() )
    {
        if ( not ozcore_image_socket_connected_ and ozcore_image_server_socket_desc_ > 0 )
        {
            ozcore_image_socket_desc_ = DriverSocket::waitConnect( ozcore_image_server_socket_desc_ );

            std::this_thread::sleep_for( 50ms );

            if ( ozcore_image_socket_desc_ > 0 )
            {
                ozcore_image_socket_connected_ = true;

                ROS_ERROR( "OzCore Image Socket Connected" );

                milliseconds image_now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                last_ozcore_image_socket_activity_time_ = static_cast<int64_t>( image_now_ms.count());
            }
        }

        if ( ozcore_image_socket_connected_ )
        {
            milliseconds image_now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
            int64_t now = static_cast<int64_t>( image_now_ms.count());

            if ( now - last_ozcore_image_socket_activity_time_ > 5000 )
            {
                ozcore_image_disconnected();
            }
            else
            {
                bool new_image_received = false;

                ozcore_image_packet_to_send_access_.lock();

                if( last_ozcore_image_sent_ms != last_ozcore_image_ms_ )
                {
                    std::memcpy( image_buffer_to_send, image_buffer_to_send_, 721920 );
                    new_image_received = true;

                    last_ozcore_image_sent_ms = last_ozcore_image_ms_;
                }

                ozcore_image_packet_to_send_access_.unlock();

                if ( new_image_received == true )
                {
                    int total_written_bytes = 0;
                    ssize_t write_size = 0;

                    int nb_tries = 0;
                    int max_tries = 500;

                    while( total_written_bytes < 721920 and nb_tries < max_tries )
                    {
                        write_size = send( ozcore_image_socket_desc_, image_buffer_to_send + total_written_bytes, 721920 - total_written_bytes, 0 );
                        std::this_thread::sleep_for( 5ms );

                        if( write_size < 0 )
                        {
                            nb_tries++;
                            std::this_thread::sleep_for( 10ms );
                        }
                        else
                        {
                            total_written_bytes = total_written_bytes + static_cast<int>( write_size );
                            nb_tries = 0;
                        }
                    }

                    if( nb_tries >= max_tries )
                    {
                        ROS_ERROR("send packet failed, too many tries");
                    }
                    else
                    {
                        ROS_INFO("Camera packet send to 5558");
                    }
                }
            }
        }

        std::this_thread::sleep_for( 5ms );
    }

    close( ozcore_image_server_socket_desc_ );

    ozcore_image_thread_started_ = false;

}

// *********************************************************************************************************************

void Core::join_client_read_thread()
{
    client_read_thread_.join();
    send_odo_thread_.join();
    image_thread_.join();
    image_read_thread_.join();

}

// *********************************************************************************************************************

void Core::send_lidar_packet_callback( const sensor_msgs::LaserScan::ConstPtr& lidar_msg )
{
    try
    {
        uint16_t distance[ 271 ];
        uint8_t albedo[ 271 ];

        for (int i = 0; i < 271; ++i)
        {
            if( i >= 45 and i < 226 )
            {
                distance[ i ] = lidar_msg->ranges[ 270- i ] * 1000; //Convert meters to millimeters
            }
            else
            {
                distance[ i ] = 0;
            }

            albedo[ i ] = 0;
        }

        HaLidarPacketPtr lidarPacketPtr = std::make_shared< HaLidarPacket >( distance, albedo );

        packet_to_send_list_access_.lock();

        packet_to_send_list_.push_back( lidarPacketPtr );

        packet_to_send_list_access_.unlock();


        ROS_INFO("Lidar packet enqueued");
    }
    catch ( SocketException& e )
    {
        ROS_ERROR( "Socket exception was caught : %s", e.description().c_str() );
    }
}


// *********************************************************************************************************************

void Core::send_actuator_position_callback( const sensor_msgs::JointState::ConstPtr& joint_states_msg )
{
    try
    {
        actuator_position_ = joint_states_msg->position[0] ;
        uint8_t position_percent = std::round(actuator_position_  * ( -100.0 / 0.15 ));

        ApiMoveActuatorPacketPtr ActuatorPositionPacketPtr = std::make_shared< ApiMoveActuatorPacket >( position_percent );

        packet_to_send_list_access_.lock();

        ROS_ERROR( "Send position : %d",position_percent);

        packet_to_send_list_.push_back( ActuatorPositionPacketPtr );

        packet_to_send_list_access_.unlock();

        ROS_INFO("Actuator position packet enqueued");
    }
    catch ( SocketException& e )
    {
        ROS_ERROR( "Socket exception was caught : %s", e.description().c_str() );
    }
}

// *********************************************************************************************************************

void Core::send_camera_packet_callback(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right)
{
    try
    {
        cl::BufferUPtr dataBuffer = cl::unique_buffer( static_cast<size_t>( 721920 ) );

        ozcore_image_packet_to_send_access_.lock();

        std::memcpy( image_buffer_to_send_, &image_left->data[ 0 ], 360960 );
        std::memcpy( image_buffer_to_send_+ 360960, &image_right->data[ 0 ], 360960 );

        std::memcpy( &(*dataBuffer)[ 0 ], &image_left->data[ 0 ], 360960 );
        std::memcpy( &(*dataBuffer)[ 0 ] + 360960, &image_right->data[ 0 ], 360960 );


        milliseconds ozcore_image_now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        last_ozcore_image_ms_ = static_cast<int64_t>( ozcore_image_now_ms.count());

        ozcore_image_packet_to_send_access_.unlock();

        image_packet_to_send_access_.lock();

        image_packet_to_send_ = std::make_shared<ApiStereoCameraPacket>( ApiStereoCameraPacket::ImageType::RAW_IMAGES, std::move( dataBuffer ) ); ;

        milliseconds image_now_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        last_image_ms_ = static_cast<int64_t>( image_now_ms.count());

        image_packet_to_send_access_.unlock();

        ROS_INFO("Stereo camera packet managed");
    }
    catch (SocketException& e )
    {
        ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
    }
}

// *********************************************************************************************************************

void Core::send_imu_packet_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    try {

        int16_t x_gyro = static_cast<int16_t>(imu_msg->angular_velocity.x * 1000.0 * 360.0 / (2.0 *  M_PI * -30.5) );
        int16_t y_gyro = static_cast<int16_t>(imu_msg->angular_velocity.y * 1000.0 * 360.0 / (2.0 *  M_PI * -30.5) );
        int16_t z_gyro = static_cast<int16_t>(imu_msg->angular_velocity.z * 1000.0 * 360.0 / (2.0 *  M_PI * -30.5) );

        HaGyroPacketPtr gyroPacketPtr = std::make_shared<HaGyroPacket>(x_gyro, y_gyro, z_gyro);

        packet_to_send_list_access_.lock();

        packet_to_send_list_.push_back(gyroPacketPtr);

        packet_to_send_list_access_.unlock();

        ROS_INFO("Gyro packet enqueued");

        int16_t x_accelero = static_cast<int16_t>(imu_msg->linear_acceleration.x * 1000.0 / 9.8);
        int16_t y_accelero = static_cast<int16_t>(imu_msg->linear_acceleration.y * 1000.0 / 9.8);
        int16_t z_accelero = static_cast<int16_t>(imu_msg->linear_acceleration.z * -1000.0 / 9.8);

        HaAcceleroPacketPtr acceleroPacketPtr = std::make_shared<HaAcceleroPacket>(x_accelero, y_accelero, z_accelero);

        packet_to_send_list_access_.lock();

        packet_to_send_list_.push_back(acceleroPacketPtr);

        packet_to_send_list_access_.unlock();

        ROS_INFO("Accelero packet enqueued");
    }
    catch (SocketException& e)
    {
        ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
    }
}

// *********************************************************************************************************************

void Core::send_gps_packet_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg )
{
    try
    {

        ulong time = gps_fix_msg->header.stamp.toSec()*1000;
        double lat = gps_fix_msg->latitude;
        double lon = gps_fix_msg->longitude;
        double alt = gps_fix_msg->altitude;
        uint8_t unit = 1;
        uint8_t satUsed = 3;
        uint8_t quality = 3;
        double groundSpeed = sqrt(gps_vel_msg->vector.x*gps_vel_msg->vector.x + gps_vel_msg->vector.y*gps_vel_msg->vector.y );

        HaGpsPacketPtr gpsPacketPtr = std::make_shared<HaGpsPacket>(time,lat,lon,alt,unit,satUsed,quality,groundSpeed);

        packet_to_send_list_access_.lock();

        packet_to_send_list_.push_back(gpsPacketPtr);

        packet_to_send_list_access_.unlock();

        ROS_INFO("Gps packet enqueued");

    }
    catch (SocketException& e)
    {
        ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
    }
}

// *********************************************************************************************************************

void Core::send_odo_packet()
{

    using namespace std::chrono_literals;
    uint8_t fr = 0;
    uint8_t br = 0;
    uint8_t bl = 0;
    uint8_t fl = 0;

    while (ros::ok()) {

        if (client_socket_connected_) {

            // Initialisation
            double pitch_bl = getPitch("/back_left_wheel");
            double pitch_fl = getPitch("/front_left_wheel");
            double pitch_br = getPitch("/back_right_wheel");
            double pitch_fr = getPitch("/front_right_wheel");

            double pitch_last_tic_bl = pitch_bl;
            int forward_backward_bl = 0;
            double pitch_last_tic_fl = pitch_fl;
            int forward_backward_fl = 0;
            double pitch_last_tic_fr = pitch_fr;
            int forward_backward_fr = 0;
            double pitch_last_tic_br = pitch_br;
            int forward_backward_br = 0;

            while (ros::ok()) {
                bool tic = false;

                while (not tic and ros::ok()) {
                    std::this_thread::sleep_for(10ms);

                    pitch_bl = getPitch("/back_left_wheel");
                    pitch_fl = getPitch("/front_left_wheel");
                    pitch_br = getPitch("/back_right_wheel");
                    pitch_fr = getPitch("/front_right_wheel");

                    tic = odo_wheel(bl, pitch_bl, pitch_last_tic_bl, forward_backward_bl);
                    tic = odo_wheel(fl, pitch_fl, pitch_last_tic_fl, forward_backward_fl) or tic;
                    tic = odo_wheel(br, pitch_br, pitch_last_tic_br, forward_backward_br) or tic;
                    tic = odo_wheel(fr, pitch_fr, pitch_last_tic_fr, forward_backward_fr) or tic;
                }

                HaOdoPacketPtr odoPacketPtr = std::make_shared<HaOdoPacket>(fr, br, bl, fl);

                packet_to_send_list_access_.lock();

                packet_to_send_list_.push_back(odoPacketPtr);

                packet_to_send_list_access_.unlock();

                ROS_INFO("Odo Status packet enqueued");
            }
        }
        else{
            fr = 0;
            br = 0;
            bl = 0;
            fl = 0;
        }
    }
}

// *********************************************************************************************************************

double Core::getPitch( std::string wheel )
{
    double roll, pitch, yaw;
    tf::StampedTransform transform;

    try {
        (*listener_ptr_).lookupTransform(wheel, "/chassis_bottom", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    transform.getBasis().getRPY(roll, pitch, yaw);

    if ( (roll == M_PI and yaw == M_PI) or (roll == -M_PI and yaw == -M_PI) )
    {
        if( pitch >= 0 )
        {
            pitch = pitch - M_PI ;
        }
        else
        {
            pitch = pitch + M_PI ;
        }
    }
    else
    {
        pitch = - pitch ;
    }
    return pitch;
}

// *********************************************************************************************************************

bool Core::odo_wheel( uint8_t & odo_wheel, double& pitch, double& pitch_last_tic, int& forward_backward)
{
    double angle_tic = 6.465/14.6;
    bool tic = false;

    if (forward_backward == 0 and pitch - pitch_last_tic > 0.001) {
        forward_backward = 1;
    } else if (forward_backward == 0 and pitch - pitch_last_tic < -0.001) {
        forward_backward = -1;
    }

    // si on a commencé par avancer
    if (forward_backward == 1 and (pitch - pitch_last_tic >= angle_tic or pitch - pitch_last_tic <= 0)) {
        pitch_last_tic = pitch;
        tic = true;
        odo_wheel = odo_wheel + 1 %2;
        forward_backward = 0;
    }

    // Si on a commencé par reculer
    if (forward_backward == -1 and (pitch - pitch_last_tic <= -angle_tic or pitch - pitch_last_tic >= 0)) {
        pitch_last_tic = pitch;
        tic = true;
        odo_wheel = odo_wheel + 1 %2;
        forward_backward = 0;
    }

    return tic;

}