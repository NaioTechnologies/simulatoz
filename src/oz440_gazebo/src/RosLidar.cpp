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

#include "createLidarTrame.hpp"
#include "RosLidar.hpp"

//==================================================================================================
// C O N S T A N T S   &   L O C A L   V A R I A B L E S

//==================================================================================================
// P I M P L   C O D E   S E T C T I O N

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
RosLidar::RosLidar( boost::asio::io_service& io_service, uint16_t port )
	: laser_topic_{ "/oz440/laser/scan" }
	, laser_queue_{ 500 }
	, ros_sub_{ }
	, acceptor_{ io_service, boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(), port ) }
	, socket_{ /*io_service*/ }
	, connected_{ }
{
	socket_ = std::make_unique< boost::asio::ip::tcp::socket >( acceptor_.get_io_service() );
	acceptor_.async_accept( *socket_, boost::bind( &RosLidar::do_accept, this,
												   boost::asio::placeholders::error ) );
}

//--------------------------------------------------------------------------------------------------
//
RosLidar::~RosLidar()
{ }

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::subscribe( ros::NodeHandle& node )
{
	ros_sub_ = node.subscribe( laser_topic_, laser_queue_, &RosLidar::ros_callback, this );
}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::cleanup()
{
	acceptor_.cancel();
}

////--------------------------------------------------------------------------------------------------
////
//void
//RosLidar::send_packet()
//{
//	char trame[10000];
//
//	int lidar[271];
//	int albedo[271];
//
//	struct timespec timeInit;
//	clock_gettime( CLOCK_MONOTONIC_RAW, &timeInit );
//
//	if( socket_connected_ )
//	{
//
//		packet_access_.lock();
//
//		if( packet_ptr_ != nullptr )
//		{
//			for( int i = 0; i < 271; i++ )
//			{
//				lidar[i] = packet_ptr_->distance[i];
//				albedo[i] = packet_ptr_->albedo[i];
//			}
//		}
//
//		packet_access_.unlock();
//
//		nbMesures_++;
//		nbTelegrammes_++;
//
//		createTrame( lidar, albedo, trame, nbMesures_, nbTelegrammes_, timeInit );
//
//		socket_access_.lock();
//		ssize_t write_size = write( socket_desc_, trame, strlen( trame ) );
//		socket_access_.unlock();
//
//		if( write_size != strlen( trame ) )
//		{
//			ROS_ERROR( "Error sending Lidar trame" );
//		}
//		else
//		{
//			ROS_INFO( "Lidar packet send" );
//		}
//	}
//}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::do_accept( const boost::system::error_code& ec )
{
	if( !ec )
	{
		ROS_ERROR_STREAM( "Connection to 'RosLidar' on 2213 successfull" );

		connected_ = true;
		//socket_.async_receive( boost::asio::buffer( receive_buffer_ ),
		//					   boost::bind( &RosLidar::do_receive, this,
		//									boost::asio::placeholders::error,
		//									boost::asio::placeholders::bytes_transferred() ) );
	}
	else
	{
		ROS_ERROR_STREAM( "Connection to 'RosLidar' on 2213 failed" << ec.message() );
	}
}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::do_receive( const boost::system::error_code& ec, std::size_t bytes_transferred )
{
	if( ec == boost::asio::error::eof || ec == boost::asio::error::connection_reset ||
		ec == boost::asio::error::broken_pipe )
	{
		connected_ = false;
		ROS_ERROR( "Connection to RosLidar closed" );

		socket_ = std::make_unique< boost::asio::ip::tcp::socket >( acceptor_.get_io_service() );
		acceptor_.async_accept( *socket_, boost::bind( &RosLidar::do_accept, this,
													   boost::asio::placeholders::error ) );
	}
	else
	{
		//if( strncmp( "\x02sRN LMDscandata 1\x03", (char*) &receive_buffer_.front(),
		//			 strlen( "\x02sRN LMDscandata 1\x03" ) ) == 0 )
		//{
		//}
	}
}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::do_send( const boost::system::error_code& ec, std::size_t bytes_transferred )
{
	if( ec == boost::asio::error::eof || ec == boost::asio::error::connection_reset ||
		ec == boost::asio::error::broken_pipe )
	{
		connected_ = false;
		ROS_ERROR( "Connection to RosLidar closed" );
		socket_ = std::make_unique< boost::asio::ip::tcp::socket >( acceptor_.get_io_service() );
		acceptor_.async_accept( *socket_, boost::bind( &RosLidar::do_accept, this,
													   boost::asio::placeholders::error ) );
	}
	else
	{ }
}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::ros_callback( const sensor_msgs::LaserScan::ConstPtr& lidar_msg )
{
	//std::lock_guard< std::mutex > lock{ datagram_access_ };
	//latest_lidar_msg_ = lidar_msg;

	if( connected_ )
	{
		uint16_t distances[271];
		uint8_t albedos[271];

		for( int i = 0; i < 271; ++i )
		{
			if( i >= 45 and i < 226 )
			{
				distances[i] = (uint16_t) (lidar_msg->ranges[270 - i] *
										   1000); //Convert meters to millimeters
			}
			else
			{
				distances[i] = 0;
			}
			albedos[i] = 0;
		}

		//struct timespec time;
		//clock_gettime( CLOCK_MONOTONIC_RAW, &time );
		//createTrame( distances, albedos, reinterpret_cast<char*>(&write_buffer_.front()), 0, 0,
		//			 time );
		
		socket_->async_send( boost::asio::buffer( write_buffer_ ),
							 boost::bind( &RosLidar::do_send, this,
										  boost::asio::placeholders::error,
										  boost::asio::placeholders::bytes_transferred() ) );
	}
}
