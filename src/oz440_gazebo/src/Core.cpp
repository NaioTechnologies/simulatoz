

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
		, log_folder_{ }
		, use_camera_{ true }
		, camera_ptr_{ nullptr }
		, camera_port_{ 5558 }
		, lidar_{ }
		, use_can_{ true }
		, can_ptr_{ nullptr }
		, can_port_{ 5559 }
		, serial_ptr_{ nullptr }
		, serial_port_{ 5554 }

		, video_folder_{ }
		, output_video_{ }
{
	ros::init( argc, argv, "Core" );

	for( int i = 1; i < argc; i++ )
	{
		if( strncmp( argv[i], "--videoFolder", 13 ) == 0 )
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
	ros::NodeHandle node;
	image_transport::ImageTransport it( node );

	boost::asio::io_service io_service;
	boost::optional< boost::asio::io_service::work > work( boost::in_place( boost::ref( io_service ) ) ) ;

	boost::thread_group worker_threads;
	for( uint32_t x = 0; x < 2; ++x )
	{
		worker_threads.create_thread( boost::bind( &boost::asio::io_service::run, &io_service ) );
	}

	lidar_ = std::make_unique< RosLidar >( io_service, 2213 );
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

	// subscribe to top_camera topic
	image_transport::Subscriber top_camera_sub = it.subscribe( "/oz440/top_camera/image_raw", 5, &Core::callback_top_camera, this );

	while( ros::master::check() )
	{
		ros::spinOnce();
		std::this_thread::sleep_for( 3ms );
	}

	serial_ptr_->cleanup();
	camera_ptr_->cleanup();
	can_ptr_->cleanup();
	lidar_->cleanup();

	work = boost::none;
	worker_threads.join_all();

	ROS_ERROR( "Core run thread stopped" );
}

// *********************************************************************************************************************

void
Core::callback_top_camera( const sensor_msgs::Image::ConstPtr& image )
{
	if( video_folder_.empty() )
	{
		bool success = setup_video_folder();

		if( !success )
		{
			//ROS_ERROR( "Error creating the video folder" );
		}
	}

	if( !output_video_.isOpened() )
	{
		std::string codec = "MJPG";
		cv::Size size( image->width, image->height );
		std::string filename = video_folder_ + "/output.avi";

		output_video_.open( filename,
							CV_FOURCC( codec.c_str()[0], codec.c_str()[1], codec.c_str()[2],
									   codec.c_str()[3] ), 5, size, true );

		if( !output_video_.isOpened() )
		{
			ROS_ERROR(
					"Could not create the output video! Check filename and/or support for codec." );
			exit( -1 );
		}
	}
}

// *********************************************************************************************************************

bool
Core::setup_video_folder()
{
	namespace fs = boost::filesystem;
	bool success{ };

	// Check if folder exists.
	if( fs::exists( log_folder_ ) )
	{
		// We create a dated folder and two subfolders to write our images.
		std::time_t t = std::time( NULL );
		size_t bufferSize{ 256 };
		char buff[bufferSize];
		std::string format{ "%F_%H:%M:%S" };
		size_t bytesRead = std::strftime( buff, bufferSize, format.c_str(), std::localtime( &t ) );

		std::string date_str = std::string( buff, bytesRead );
		std::replace( date_str.begin(), date_str.end(), ':', '_' );

		fs::path dated_folder_path{ log_folder_ };
		dated_folder_path /= (date_str);

		// Creating the folders.
		if( !fs::exists( dated_folder_path ) )
		{
			fs::create_directory( dated_folder_path );
			video_folder_ = dated_folder_path.c_str();
		}

		success = true;
	}
	else
	{
		//ROS_ERROR( "Log_video folder does not exist" );
	}

	return success;
}
