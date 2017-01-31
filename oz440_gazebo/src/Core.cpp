#include "../include/Core.hpp"
#include "DriverSocket.hpp"
#include <chrono>
#include <pthread.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <geometry_msgs/PointStamped.h>

#include "oz440_socket/SocketException.h"

#include "oz440_api/HaLidarPacket.hpp"
#include "oz440_api/HaMotorsPacket.hpp"
#include "oz440_api/HaGpsPacket.hpp"
#include "oz440_api/HaGyroPacket.hpp"
#include "oz440_api/HaAcceleroPacket.hpp"
#include "oz440_api/ApiStereoCameraPacket.hpp"
#include "oz440_api/HaOdoPacket.hpp"
#include "oz440_api/ApiMoveActuatorPacket.hpp"

#include <errno.h>
#include <vector>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <signal.h>
#include <std_srvs/Empty.h>

#include "../include/Bridge.hpp"

#define IMAGE_SIZE 752*480
#define HEADER_SIZE 15

using namespace std::chrono;

static pid_t gettid( void )
{
    return syscall( __NR_gettid );
}

std::atomic<bool> terminate_;


// *********************************************************************************************************************

int main( int argc, char **argv )
{
    Core core( argc, argv );

    core.run(argc, argv);

    return 0;
}

// *********************************************************************************************************************

Core::Core( int argc, char **argv )
    : use_camera_ {true}
    , camera_ptr_ {nullptr}
    , camera_port_ {5558}
    , use_lidar_ {true}
    , lidar_ptr_ {nullptr}
    , lidar_port_ {2213}
    , use_can_ {true}
    , terminate_ {false}
    , received_packet_list_ {}
    , read_thread_started_ {false}
    , read_thread_ {}
    , send_odo_thread_ {}
    , bridge_thread_started_ {false}
    , bridge_thread_ {}
    , graphics_on_ {true}
    , actuator_position_ {0.0}
{
    ros::init( argc, argv, "Core");

    ros::NodeHandle n;

    listener_ptr_ = std::make_shared< tf::TransformListener>( ros::Duration(10) );

    // Bridge initialisation
    bridge_ptr_ = std::make_shared<Bridge>();
}

// *********************************************************************************************************************

Core::~Core( )
{
}

// *********************************************************************************************************************

void Core::run( int argc, char **argv )
{
    using namespace std::chrono_literals;
    bool last_order_down = 0;

    std::this_thread::sleep_for(1500ms);

    ros::NodeHandle n;

    // Create motors and actuator commands publishers
    velocity_pub_ = n.advertise<geometry_msgs::Vector3>( "/oz440/cmd_vel", 10 );
    actuator_pub_ = n.advertise<geometry_msgs::Vector3>( "/oz440/cmd_act", 10 );

    if( use_lidar_ )
    {
        lidar_ptr_ = std::make_shared<Lidar>(lidar_port_);
    }

    if( use_camera_ )
    {
        camera_ptr_ = std::make_shared<Camera>(camera_port_);
    }

    ros::Subscriber lidar_sub = n.subscribe("/oz440/laser/scan", 500, &Core::callback_lidar, this);

    // subscribe to actuator position
    ros::Subscriber actuator_position_sub = n.subscribe( "/oz440/joint_states", 500, &Core::callback_actuator_position, this );

    // subscribe to imu topic
    ros::Subscriber imu_sub = n.subscribe("/oz440/imu/data", 50, &Core::callback_imu, this);

    // subscribe to gps topic
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_fix_sub( n, "/oz440/navsat/fix", 5 );
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gps_vel_sub( n, "/oz440/navsat/vel", 5 );
    message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::Vector3Stamped> sync_gps( gps_fix_sub, gps_vel_sub, 10 );
    sync_gps.registerCallback( boost::bind( &Core::callback_gps, this, _1, _2 ) );

    message_filters::Subscriber<sensor_msgs::Image> image_left_sub ( n, "/oz440/camera/left/image_raw", 1 );
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub ( n, "/oz440/camera/right/image_raw", 1 );
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_left_sub, image_right_sub, 10);
    sync.registerCallback ( boost::bind(&Core::callback_camera, this, _1, _2) );

//    std_srvs::Empty message;
//    ros::service::call("gazebo/reset_world", message);

    // create_odo_thread
    send_odo_thread_ = std::thread( &Core::odometry_thread, this );

    // create_read_thread
    read_thread_ = std::thread( &Core::read_thread_function, this );

    // create_bridge_thread
    bridge_thread_ = std::thread( &Core::bridge_thread_function, this );

    while ( ros::master::check() and !terminate_ )
    {
        std::this_thread::sleep_for(5ms);
        ros::spinOnce();
    }

    bridge_ptr_-> stop_main_thread_asked();
    camera_ptr_->ask_stop();
    lidar_ptr_->ask_stop();

    terminate_ = true;

    send_odo_thread_.join();
    bridge_thread_.join();

    std::this_thread::sleep_for(500ms);
    ROS_ERROR("Core run thread stopped");
}

// *********************************************************************************************************************

void Core::read_thread_function( )
{
    using namespace std::chrono_literals;

    bool last_order_down = 0;

    read_thread_started_ = true;

    try
    {
        while ( !terminate_ )
        {
            bool stop_main_thread_asked = bridge_ptr_->get_stop_main_thread_asked();

            if(stop_main_thread_asked)
            {
                terminate_ = true;
                ros::shutdown();
            }

            received_packet_list_ = bridge_ptr_->get_packet_list_to_send();

            for ( auto &&packetPtr : received_packet_list_) // For every packet decoded
            {
                if (std::dynamic_pointer_cast<HaMotorsPacket>(packetPtr)) {

                    //  When receiving a motor order
                    HaMotorsPacketPtr motorsPacketPtr = std::dynamic_pointer_cast<HaMotorsPacket>(packetPtr);

                    double rightspeed = static_cast<double>(motorsPacketPtr->right);
                    double leftspeed = static_cast<double>(motorsPacketPtr->left);

                    ROS_INFO("ApiMotorsPacket received, right: %f left : %f", rightspeed, leftspeed);

                    geometry_msgs::Vector3 command;

                    command.x = ((leftspeed / 127.0) * 3.4);
                    command.y = ((rightspeed / 127.0) * 3.4);

                    velocity_pub_.publish(command);
                }
//                else if (std::dynamic_pointer_cast<ApiMoveActuatorPacket>(packetPtr))
//                {
//                    //  When receiving an actuator order
//                    ApiMoveActuatorPacketPtr ActuatorPacketPtr = std::dynamic_pointer_cast<ApiMoveActuatorPacket>(packetPtr);
//
//                    geometry_msgs::Vector3 command;
//
//                    if ( ActuatorPacketPtr->position == 1 )
//                    {
//                        command.x = actuator_position_ +0.005;
//                        ROS_INFO("MONTE : %f", actuator_position_ +0.005);
//                        last_order_down = 0;
//
//                    }
//                    else if ( ActuatorPacketPtr->position == 2 )
//                    {
//                        command.x = actuator_position_ -0.005;
//                        ROS_INFO("DESCEND : %f", actuator_position_ -0.005);
//                        last_order_down = 1;
//                    }
//                    else
//                    {
//                        if (last_order_down == 1) {
//                            command.x = actuator_position_ - 0.0001;
//                            std::this_thread::sleep_for(10ms);
//                        }
//                        else
//                        {
//                            command.x = actuator_position_ + 0.0001;
//                            std::this_thread::sleep_for(10ms);
//                        }
//                    }
//                    ROS_INFO("ApiMoveActuatorPacket received, position: %f ", command.x);
//
//                    actuator_pub_.publish(command);
//                }
                received_packet_list_.clear();
                std::this_thread::sleep_for(5ms);

            }
        }
    }
    catch (SocketException& e )
    {
        ROS_INFO( "Read_thread_function exception was caught : %s", e.description().c_str() );
    }
    read_thread_started_ = false;
}

//**********************************************************************************************************************

void Core::bridge_thread_function()
{
    using namespace std::chrono_literals;

    std::this_thread::sleep_for( 3000ms );

    bridge_ptr_->init(graphics_on_);

    while(!terminate_)
    {
        std::this_thread::sleep_for( 500ms );
    }
}

// *********************************************************************************************************************

void Core::callback_lidar( const sensor_msgs::LaserScan::ConstPtr& lidar_msg )
{
    try
    {
        if( use_lidar_ and lidar_ptr_->connected() )
        {
            uint16_t distance[271];
            uint8_t albedo[271];

            for (int i = 0; i < 271; ++i) {
                if (i >= 45 and i < 226) {
                    distance[i] = lidar_msg->ranges[270 - i] * 1000; //Convert meters to millimeters
                } else {
                    distance[i] = 0;
                }

                albedo[i] = 0;
            }

            HaLidarPacketPtr lidarPacketPtr = std::make_shared<HaLidarPacket>(distance, albedo);

            lidar_ptr_->set_packet(lidarPacketPtr);

            ROS_INFO("Lidar packet enqueued");
        }
    }
    catch ( SocketException& e )
    {
        ROS_ERROR( "Socket exception was caught : %s", e.description().c_str() );
    }
}


// *********************************************************************************************************************

void Core::callback_actuator_position( const sensor_msgs::JointState::ConstPtr& joint_states_msg )
{
    try
    {
        actuator_position_ = joint_states_msg->position[0] ;
        uint8_t position_percent = std::round(actuator_position_  * ( -100.0 / 0.15 ));

        ApiMoveActuatorPacketPtr ActuatorPositionPacketPtr = std::make_shared< ApiMoveActuatorPacket >( position_percent );

        bridge_ptr_->add_received_packet(ActuatorPositionPacketPtr);

        ROS_INFO("Actuator position packet enqueued");
    }
    catch ( SocketException& e )
    {
        ROS_ERROR( "Socket exception was caught : %s", e.description().c_str() );
    }
}

// *********************************************************************************************************************

void Core::callback_camera(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right)
{
    try
    {
        if( use_camera_ and camera_ptr_->connected() )
        {
            std::array < uint8_t, 721920 > image_buffer_to_send;

            std::memcpy( image_buffer_to_send.data(), &image_left->data[ 0 ], 360960 );
            std::memcpy( image_buffer_to_send.data() + 360960, &image_right->data[ 0 ], 360960 );

            camera_ptr_->set_image(image_buffer_to_send);

            ROS_INFO("Stereo camera packet managed");
        }
    }
    catch (SocketException& e )
    {
        ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
    }
}

// *********************************************************************************************************************

void Core::callback_imu(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    try {

        int16_t x_gyro = static_cast<int16_t>(imu_msg->angular_velocity.x * 1000.0 * 360.0 / (2.0 *  M_PI * -30.5) );
        int16_t y_gyro = static_cast<int16_t>(imu_msg->angular_velocity.y * 1000.0 * 360.0 / (2.0 *  M_PI * -30.5) );
        int16_t z_gyro = static_cast<int16_t>(imu_msg->angular_velocity.z * 1000.0 * 360.0 / (2.0 *  M_PI * -30.5) );

        HaGyroPacketPtr gyroPacketPtr = std::make_shared<HaGyroPacket>(x_gyro, y_gyro, z_gyro);

        bridge_ptr_->add_received_packet(gyroPacketPtr);

        ROS_INFO("Gyro packet enqueued");

        int16_t x_accelero = static_cast<int16_t>(imu_msg->linear_acceleration.x * 1000.0 / 9.8);
        int16_t y_accelero = static_cast<int16_t>(imu_msg->linear_acceleration.y * 1000.0 / 9.8);
        int16_t z_accelero = static_cast<int16_t>(imu_msg->linear_acceleration.z * -1000.0 / 9.8);

        HaAcceleroPacketPtr acceleroPacketPtr = std::make_shared<HaAcceleroPacket>(x_accelero, y_accelero, z_accelero);

        bridge_ptr_->add_received_packet(acceleroPacketPtr);

        ROS_INFO("Accelero packet enqueued");
    }
    catch (SocketException& e)
    {
        ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
    }
}

// *********************************************************************************************************************

void Core::callback_gps(const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg )
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

        bridge_ptr_->add_received_packet(gpsPacketPtr);

        ROS_INFO("Gps packet enqueued");

    }
    catch (SocketException& e)
    {
        ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
    }
}

// *********************************************************************************************************************

void Core::odometry_thread()
{
    using namespace std::chrono_literals;
    uint8_t fr = 0;
    uint8_t br = 0;
    uint8_t bl = 0;
    uint8_t fl = 0;

    std::this_thread::sleep_for(3000ms);

    while (!terminate_) {

        if (bridge_ptr_->get_can_connected_()) {

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

            while (!terminate_) {
                bool tic = false;

                while (not tic and !terminate_) {

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

                bridge_ptr_->add_received_packet(odoPacketPtr);
            }
        }
        else{
            fr = 0;
            br = 0;
            bl = 0;
            fl = 0;

            std::this_thread::sleep_for(100ms);
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