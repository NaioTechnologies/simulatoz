

#include <chrono>

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <boost/utility.hpp>
#include <boost/none.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "Core.hpp"
#include "RosLidar.hpp"

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
        , lidar_{ }
        , use_can_{ true }
        , can_ptr_{ nullptr }
        , can_port_{ 5559 }
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

// *********************************************************************************************************************

Core::~Core()
{
}

// *********************************************************************************************************************

void
Core::run()
{
    ROS_ERROR(" RUN");

    using namespace std::chrono_literals;
    std::this_thread::sleep_for( 1500ms );
    ros::NodeHandle node;
    image_transport::ImageTransport it( node );

	boost::asio::io_service io_service;
	boost::optional< boost::asio::io_service::work > work( boost::in_place( boost::ref( io_service ) ) ) ;

    boost::thread_group worker_threads;
    for( uint32_t x = 0; x < 2; ++x )
    {
        worker_threads.create_thread( boost::bind( &boost::asio::io_service::run, &io_service ) );
    }

    lidar_ = std::make_unique< Lidar >( 2213 );
    lidar_->subscribe( node );

	if( use_camera_ )
	{
		camera_ptr_ = std::make_shared< Camera >( camera_port_ );
		camera_ptr_->subscribe( node );
	}

	if( use_can_ )
	{
		can_ptr_ = std::make_shared< Can >( can_port_ );
		can_ptr_->subscribe( node );

	}

	serial_ptr_ = std::make_shared< Serial >( serial_port_ );
	serial_ptr_->advertise( node );

    if( video_log_folder_ != "NO_VIDEO" )
    {
        video_log_ptr_ = std::make_shared< VideoLog >( video_log_folder_ );
        video_log_ptr_->subscribe( it );
    }

    if( log_folder_ != "NO_LOG" )
    {
        log_ptr_ = std::make_shared< Log >( log_folder_ );
        metric_ptr_ = std::make_shared< Metric >( log_ptr_ );
        metric_ptr_->subscribe( node );
    }

    while( ros::master::check() )
    {
        ros::spinOnce();
        std::this_thread::sleep_for( 3ms );
    }
    metric_ptr_->cleanup();

	serial_ptr_->cleanup();
	camera_ptr_->cleanup();
	can_ptr_->cleanup();
	lidar_->cleanup();

	work = boost::none;
	worker_threads.join_all();

	ROS_ERROR( "Core run thread stopped" );
}
