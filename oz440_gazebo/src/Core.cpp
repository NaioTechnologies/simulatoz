#include "Core.hpp"

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

#include <vector>

#define IMAGE_SIZE 752*480
#define HEADER_SIZE 15

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

    stop_client_read_thread_asked_ = false;

    client_read_thread_started_ = false;
}

// *********************************************************************************************************************

Core::~Core( )
{
}

// *********************************************************************************************************************

void Core::run( )
{
    ros::NodeHandle n;

    velocity_pub_ = n.advertise<geometry_msgs::Vector3>( "/oz440/cmd_vel", 10 );

    //ros::Rate loop_rate(10);

    // subscribe to lidar topic
    ros::Subscriber lidar_sub = n.subscribe( "/oz440/laser/scan", 500000, &Core::send_lidar_packet_callback, this );

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

    server_socket_ptr_ = new ServerSocket( naio01_server_port );
    accepted_socket_ptr_ = new ServerSocket( );

    server_socket_ptr_->accept( *accepted_socket_ptr_ );

    ROS_INFO( "Connexion Socket");

    // creates main thread
    client_read_thread_ = std::thread( &Core::client_read_thread_function, this );

    // create_odo_thread
    send_odo_thread_ = std::thread( &Core::send_odo_packet, this );

    // send packets
    while ( ros::ok() )
    {
        packet_to_send_list_access_.lock();

        for( BaseNaio01PacketPtr packet : packet_to_send_list_ )
        {
            cl::BufferUPtr buffer = packet->encode();
            accepted_socket_ptr_->sendToSock( buffer->data(), buffer->size() );
        }

        packet_to_send_list_.clear();

        packet_to_send_list_access_.unlock();
    }
}

// *********************************************************************************************************************

void Core::client_read_thread_function( )
{
    using namespace std::chrono_literals;

    int size;
    uint8_t received_buffer[ 4096 ];
    bool packet_header_detected = false;
    bool at_least_one_packet_decoded = false;

    Naio01Codec naio_01_codec;
    std::vector< BaseNaio01PacketPtr > received_packet_list;

    client_read_thread_started_ = true;

    try
    {
        while ( !stop_client_read_thread_asked_ and ros::ok() )
        {
            ROS_INFO("Loop start");
            size = accepted_socket_ptr_->recvFromSock( received_buffer );

            if (size > 0)
            {
                // Try to decode the received data
                at_least_one_packet_decoded = naio_01_codec.decode( received_buffer, size, packet_header_detected );

                ROS_INFO("at least one packet decoded");

                naio_01_codec.reset();
            }

            if ( at_least_one_packet_decoded )
            {
                // If successfull decoding emplace packets in the received buffer
                for ( uint i = 0; i < naio_01_codec.currentBasePacketList.size() ; i++ )
                {
                    received_packet_list.emplace_back( naio_01_codec.currentBasePacketList[i] );
                }

                naio_01_codec.currentBasePacketList.clear();

                for ( uint i = 0; i < received_packet_list.size(); i++ ) // For every packet decoded
                {
                    BaseNaio01PacketPtr basePacketPtr = received_packet_list.at( i );

                    if ( std::dynamic_pointer_cast<HaMotorsPacket>( basePacketPtr ) )
                    {
                        //  When receiving a motor order
                        HaMotorsPacketPtr motorsPacketPtr = std::dynamic_pointer_cast<HaMotorsPacket>(basePacketPtr);
                        double rightspeed = static_cast<double>(motorsPacketPtr->right);
                        double leftspeed = static_cast<double>(motorsPacketPtr->left);
                        ROS_INFO("ApiMotorsPacket received, right: %f left : %f", rightspeed, leftspeed);

                        geometry_msgs::Vector3 command ;

                        command.x = ( ( leftspeed / 127.0 ) * 3.4 );
                        command.y = ( ( rightspeed / 127.0) * 3.4 );

                        velocity_pub_.publish( command );
                    }
                }

                received_packet_list.clear();
            }
            std::this_thread::sleep_for(10ms);
            ros::spinOnce();
        }
    }
    catch (SocketException& e )
    {
        ROS_ERROR( "client_read_thread_function exception was caught : %s", e.description().c_str() );
    }

    stop_client_read_thread_asked_ = false;

    client_read_thread_started_ = false;
}

// *********************************************************************************************************************

void Core::join_client_read_thread()
{
    client_read_thread_.join();
    send_odo_thread_.join();
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

void Core::send_camera_packet_callback(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right)
{
  try
  {
      cl::BufferUPtr dataBuffer = cl::unique_buffer( static_cast<size_t>( 2*IMAGE_SIZE ) );

      for ( int i = 0 ; i < IMAGE_SIZE ; i++ )
      {
          (*dataBuffer)[ i ] = image_left->data[ i ];
          (*dataBuffer)[ i + IMAGE_SIZE ] = image_right->data[ i ];
      }

     ApiStereoCameraPacketPtr stereoCameraPacketPtr = std::make_shared<ApiStereoCameraPacket>( ApiStereoCameraPacket::ImageType::RAW_IMAGES, std::move( dataBuffer ) );

     packet_to_send_list_access_.lock();

      packet_to_send_list_.push_back( stereoCameraPacketPtr );

      packet_to_send_list_access_.unlock();

      ROS_INFO("Stereo camera packet enqueued");
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

void Core::send_odo_packet() {

    using namespace std::chrono_literals;
    uint8_t fr = 0;
    uint8_t br = 0;
    uint8_t bl = 0;
    uint8_t fl = 0;

    std::this_thread::sleep_for(1s);

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

    while (ros::ok())
    {
        bool tic = false;

        while (not tic and ros::ok())
        {
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

// *********************************************************************************************************************

double Core::getPitch( std::string wheel ){
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