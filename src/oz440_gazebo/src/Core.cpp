
#include "Core.hpp"

#include <chrono>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

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
        : use_camera_{ true }
        , camera_ptr_{ nullptr }
        , camera_port_{ 5558 }
        , use_lidar_{ true }
        , lidar_ptr_{ nullptr }
        , lidar_port_{ 2213 }
        , use_can_{ true }
        , can_ptr_{ nullptr }
        , can_port_{ 5559 }
        , odometry_ptr_{ nullptr }
        , serial_ptr_{ nullptr }
        , serial_port_{ 5554 }
        , actuator_position_{ 0.0 }
        , video_log_ptr_ { nullptr }
        , video_log_folder_ { }
        , log_ptr_ { nullptr }
        , log_folder_ { }
{
    ros::init( argc, argv, "Core" );

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
        can_ptr_->init();
        can_ptr_->subscribe( n );

        odometry_ptr_ = std::make_shared< Odometry >( can_ptr_ );
        odometry_ptr_->init();
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

    while( ros::master::check() )
    {
        std::this_thread::sleep_for( 3ms );
        ros::spinOnce();
    }

    camera_ptr_->cleanup();
    lidar_ptr_->cleanup();
    can_ptr_->cleanup();
    serial_ptr_->cleanup();

    odometry_ptr_->cleanup();

    std::this_thread::sleep_for( 500ms );

    ROS_ERROR( "Core run thread stopped" );

}