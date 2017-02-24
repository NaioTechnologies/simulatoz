//==================================================================================================
//
//  Copyright(c)  2016  Naio Technologies. All rights reserved.
//
//  These coded instructions, statements, and computer programs contain unpublished proprietary
//  information written by Naio Technologies and are protected by copyright law. They may not be
//  disclosed to third parties or copied or duplicated in any form, in whole or in part, without
//  the prior written consent of Naio Technologies.
//
//==================================================================================================

//==================================================================================================
// I N C L U D E   F I L E S

#include "Core.hpp"

#include <chrono>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

using namespace std::chrono;

//==================================================================================================
// C O N S T A N T S   &   L O C A L   V A R I A B L E S

//==================================================================================================
// M A I N   C O D E   S E T C T I O N

int
main( int argc, char** argv )
{
    Core core( argc, argv );

    core.run();

    return 0;
}

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

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

Core::~Core()
{
}


//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

void
Core::run()
{
    using namespace std::chrono_literals;

    std::this_thread::sleep_for( 1500ms );

    ros::NodeHandle node;

    image_transport::ImageTransport it( node );

    if( video_log_folder_ != "NO_VIDEO" )
    {
        video_log_ptr_ = std::make_shared< VideoLog >( video_log_folder_ );
        video_log_ptr_->subscribe( it );
    }

    if( log_folder_ != "NO_LOG" )
    {
        log_ptr_ = std::make_shared<Log>(log_folder_);
        metric_ptr_ = std::make_shared<Metric>(log_ptr_);
        metric_ptr_->subscribe(node);
    }

    if( use_lidar_ )
    {
        lidar_ptr_ = std::make_shared< Lidar >( lidar_port_ );
        lidar_ptr_->subscribe( node );
    }

    if( use_camera_ )
    {
        camera_ptr_ = std::make_shared< Camera >( camera_port_ );
        camera_ptr_->subscribe( node );
    }

    if( use_can_ )
    {
        can_ptr_ = std::make_shared< Can >( can_port_, log_ptr_ );
        can_ptr_->init();
        can_ptr_->subscribe( node );

        odometry_ptr_ = std::make_shared< Odometry >( can_ptr_ );
        odometry_ptr_->init();
    }

    serial_ptr_ = std::make_shared< Serial >( serial_port_ );
    serial_ptr_->advertise( node );

    while( ros::master::check() )
    {
        std::this_thread::sleep_for( 3ms );
        ros::spinOnce();
    }

    camera_ptr_->cleanup();

    lidar_ptr_->cleanup();

    can_ptr_->cleanup();
    odometry_ptr_->cleanup();

    serial_ptr_->cleanup();

    metric_ptr_->cleanup();
}