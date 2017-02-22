
#include "Core.hpp"

#include <chrono>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using namespace std::chrono;

int
main( int argc, char** argv )
{
    Core core( argc, argv );

    core.run();

    return 0;
}

// *********************************************************************************************************************

Core::Core( int argc, char** argv )
        : terminate_{ false }
        , use_camera_{ true }
        , camera_ptr_{ nullptr }
        , camera_port_{ 5558 }
        , use_lidar_{ true }
        , lidar_ptr_{ nullptr }
        , lidar_port_{ 2213 }
        , use_can_{ true }
        , can_ptr_{ nullptr }
        , can_port_{ 5559 }
        , serial_ptr_{ nullptr }
        , serial_port_{ 5554 }
        , read_thread_started_{ false }
        , read_thread_{ }
        , odometry_thread_{ }
        , odometry_thread_started_{ false }
        , actuator_position_{ 0.0 }
        , video_log_ptr_ { nullptr }
        , video_log_folder_ { }
        , log_ptr_ { nullptr }
        , log_folder_ { }
{
    ros::init( argc, argv, "Core" );

    listener_ptr_ = std::make_shared< tf::TransformListener >( ros::Duration( 10 ) );

    for( int i = 1; i < argc; i++ )
    {
        if( strncmp( argv[i], "--videoFolder", 13 ) == 0 )
        {
            video_log_folder_ = argv[i + 1];
        }

        if( strncmp( argv[i], "--logFolder", 11 ) == 0 )
        {
            log_folder_ = argv[i + 1];
        }
    }
}

// *********************************************************************************************************************

Core::~Core()
{
}

// *********************************************************************************************************************

void
Core::run()
{
    using namespace std::chrono_literals;

    std::this_thread::sleep_for( 1500ms );

    ros::NodeHandle n;

    image_transport::ImageTransport it( n );

    if( use_lidar_ )
    {
        lidar_ptr_ = std::make_shared< Lidar >( lidar_port_ );
        lidar_ptr_->subscribe( n );
    }

    if( use_camera_ )
    {
        camera_ptr_ = std::make_shared< Camera >( camera_port_ );
        camera_ptr_->subscribe( n );

    }

    if( use_can_ )
    {

        can_ptr_ = std::make_shared< Can >( can_port_ );
    }

    serial_ptr_ = std::make_shared< Serial >( serial_port_ );
    serial_ptr_->advertise( n );

    if( video_log_folder_ != "NO_VIDEO" )
    {
        video_log_ptr_ = std::make_shared< VideoLog >( video_log_folder_ );
        video_log_ptr_->subscribe( it );
    }

    if( log_folder_ != "NO_LOG" )
    {
        log_ptr_ = std::make_shared< Log >( log_folder_ );
        metric_ptr_ = std::make_shared< Metric >( log_ptr_ );
        metric_ptr_->subscribe( n );
    }

    // Create motors and actuator commands publishers
    actuator_pub_ = n.advertise< geometry_msgs::Vector3 >( "/oz440/cmd_act", 10 );

    // subscribe to actuator position
    ros::Subscriber actuator_position_sub = n.subscribe( "/oz440/joint_states", 500, &Core::callback_actuator_position, this );

    // subscribe to imu topic
    ros::Subscriber imu_sub = n.subscribe( "/oz440/imu/data", 50, &Core::callback_imu, this );

    // subscribe to gps topic
    message_filters::Subscriber< sensor_msgs::NavSatFix > gps_fix_sub( n, "/oz440/navsat/fix", 5 );
    message_filters::Subscriber< geometry_msgs::Vector3Stamped > gps_vel_sub( n, "/oz440/navsat/vel", 5 );
    message_filters::TimeSynchronizer< sensor_msgs::NavSatFix, geometry_msgs::Vector3Stamped > sync_gps( gps_fix_sub, gps_vel_sub, 10 );
    sync_gps.registerCallback( boost::bind( &Core::callback_gps, this, _1, _2 ) );

    if( use_can_ )
    {
        odometry_thread_ = std::thread( &Core::odometry_thread, this );
    }

    while( ros::master::check() and !terminate_ )
    {
        std::this_thread::sleep_for( 3ms );
        ros::spinOnce();
    }

    camera_ptr_->cleanup();
    lidar_ptr_->cleanup();
    can_ptr_->ask_stop();
    serial_ptr_->cleanup();

    terminate_ = true;

    std::this_thread::sleep_for( 500ms );

    ROS_ERROR( "Core run thread stopped" );

}


// *********************************************************************************************************************

void
Core::callback_actuator_position( const sensor_msgs::JointState::ConstPtr& joint_states_msg )
{
    if( use_can_ and can_ptr_->connected() )
    {
        uint8_t position_percent = (uint8_t) (std::round(joint_states_msg->position[0] * (-100.0 / 0.15)));

        can_ptr_->add_actuator_position( position_percent );

        ROS_INFO( "Actuator position packet enqueued" );
    }
}

// *********************************************************************************************************************

void
Core::callback_imu( const sensor_msgs::Imu::ConstPtr& imu_msg )
{
    if( use_can_ and can_ptr_->connected() )
    {

        std::array<int16_t, 3> gyro_packet;

        gyro_packet.at(0) = static_cast<int16_t>(imu_msg->angular_velocity.x * 1000.0 * 360.0 /
                                              (2.0 * M_PI * -30.5));
        gyro_packet.at(1) = static_cast<int16_t>(imu_msg->angular_velocity.y * 1000.0 * 360.0 /
                                              (2.0 * M_PI * -30.5));
        gyro_packet.at(2) = static_cast<int16_t>(imu_msg->angular_velocity.z * 1000.0 * 360.0 /
                                              (2.0 * M_PI * -30.5));

        can_ptr_->add_gyro_packet( gyro_packet );

        ROS_INFO( "Gyro packet enqueued" );

        std::array<int16_t, 3> accelero_packet;

        accelero_packet.at(0) = static_cast<int16_t>(imu_msg->linear_acceleration.x * 1000.0 /
                                                  9.8);
        accelero_packet.at(1) = static_cast<int16_t>(imu_msg->linear_acceleration.y * 1000.0 /
                                                  9.8);
        accelero_packet.at(2) = static_cast<int16_t>(imu_msg->linear_acceleration.z * -1000.0 /
                                                  9.8);

        can_ptr_->add_accelero_packet( accelero_packet );

        ROS_INFO( "Accelero packet enqueued" );
    }
}

// *********************************************************************************************************************

void
Core::callback_gps( const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg,
                    const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg )
{
    if( use_can_ and can_ptr_->connected() )
    {
        struct Can::Gps_packet gps_packet;

        gps_packet.lat = gps_fix_msg->latitude;
        gps_packet.lon = gps_fix_msg->longitude;
        gps_packet.alt = gps_fix_msg->altitude;
        gps_packet.satUsed = 3;
        gps_packet.quality = 3;
        gps_packet.groundSpeed = sqrt(gps_vel_msg->vector.x * gps_vel_msg->vector.x +
                                      gps_vel_msg->vector.y * gps_vel_msg->vector.y);
        gps_packet.updated = true;

        can_ptr_->add_gps_packet( gps_packet );

        ROS_INFO( "Gps packet enqueued" );
    }
}
// *********************************************************************************************************************

void
Core::odometry_thread()
{

    odometry_thread_started_ = true;

    std::array<bool, 4> ticks{ { false, false, false, false} };

    using namespace std::chrono_literals;

    std::this_thread::sleep_for( 3000ms );

    while( !terminate_ )
    {

        if( can_ptr_->connected() )
        {
            // Initialisation
            double pitch_bl = getPitch( "/back_left_wheel" );
            double pitch_fl = getPitch( "/front_left_wheel" );
            double pitch_br = getPitch( "/back_right_wheel" );
            double pitch_fr = getPitch( "/front_right_wheel" );

            double pitch_last_tic_bl = pitch_bl;
            int forward_backward_bl = 0;
            double pitch_last_tic_fl = pitch_fl;
            int forward_backward_fl = 0;
            double pitch_last_tic_fr = pitch_fr;
            int forward_backward_fr = 0;
            double pitch_last_tic_br = pitch_br;
            int forward_backward_br = 0;

            while( !terminate_ )
            {
                bool tic = false;

                while( not tic and !terminate_ )
                {

                    std::this_thread::sleep_for( 10ms );

                    pitch_bl = getPitch( "/back_left_wheel" );
                    pitch_fl = getPitch( "/front_left_wheel" );
                    pitch_br = getPitch( "/back_right_wheel" );
                    pitch_fr = getPitch( "/front_right_wheel" );

                    tic = odo_wheel( ticks[3], pitch_bl, pitch_last_tic_bl, forward_backward_bl );
                    tic = odo_wheel( ticks[1], pitch_fl, pitch_last_tic_fl, forward_backward_fl ) or tic;
                    tic = odo_wheel( ticks[2], pitch_br, pitch_last_tic_br, forward_backward_br ) or tic;
                    tic = odo_wheel( ticks[0], pitch_fr, pitch_last_tic_fr, forward_backward_fr ) or tic;
                }

                can_ptr_->add_odo_packet( ticks );

            }
        }
        else
        {
            ticks = { { false, false, false, false} };

            std::this_thread::sleep_for( 100ms );
        }
    }
    odometry_thread_started_ = false;
}

// *********************************************************************************************************************

double
Core::getPitch( std::string wheel )
{
    double roll, pitch, yaw;
    tf::StampedTransform transform;

    try
    {
        (*listener_ptr_).lookupTransform( wheel, "/chassis_bottom", ros::Time( 0 ), transform );
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR( "%s", ex.what() );
    }

    transform.getBasis().getRPY( roll, pitch, yaw );

    if( (roll == M_PI and yaw == M_PI) or (roll == -M_PI and yaw == -M_PI) )
    {
        if( pitch >= 0 )
        {
            pitch = pitch - M_PI;
        }
        else
        {
            pitch = pitch + M_PI;
        }
    }
    else
    {
        pitch = -pitch;
    }
    return pitch;
}

// *********************************************************************************************************************

bool
Core::odo_wheel( bool& odo_wheel, double& pitch, double& pitch_last_tic, int& forward_backward )
{
    double angle_tic = 6.465 / 14.6;
    bool tic = false;

    if( forward_backward == 0 and pitch - pitch_last_tic > 0.001 )
    {
        forward_backward = 1;
    }
    else if( forward_backward == 0 and pitch - pitch_last_tic < -0.001 )
    {
        forward_backward = -1;
    }

    // si on a commencé par avancer
    if( forward_backward == 1 and
        (pitch - pitch_last_tic >= angle_tic or pitch - pitch_last_tic <= 0) )
    {
        pitch_last_tic = pitch;
        tic = true;
        odo_wheel = !odo_wheel;
        forward_backward = 0;
    }

    // Si on a commencé par reculer
    if( forward_backward == -1 and
        (pitch - pitch_last_tic <= -angle_tic or pitch - pitch_last_tic >= 0) )
    {
        pitch_last_tic = pitch;
        tic = true;
        odo_wheel = !odo_wheel;
        forward_backward = 0;
    }

    return tic;
}