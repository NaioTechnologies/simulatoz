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
	, socket_{ }
	, connected_{ }
{
	reset();
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

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::reset()
{
	connected_ = false;
	socket_ = std::make_unique< boost::asio::ip::tcp::socket >( acceptor_.get_io_service() );
	acceptor_.async_accept( *socket_, boost::bind( &RosLidar::do_accept, this,
												   boost::asio::placeholders::error ) );
}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::do_accept( const boost::system::error_code& ec )
{
	if( !ec )
	{
		ROS_ERROR( "RosLidar connection successfull" );
		connected_ = true;

		//socket_.async_receive( boost::asio::buffer( receive_buffer_ ),
		//					   boost::bind( &RosLidar::do_receive, this,
		//									boost::asio::placeholders::error,
		//									boost::asio::placeholders::bytes_transferred() ) );
	}
	else
	{
		ROS_ERROR_STREAM( "RosLidar connection failed: " << ec.message() );
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
		ROS_ERROR( "RosLidar connection closed" );
		reset();
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
		ROS_ERROR( "RosLidar connection closed" );
		reset();
	}

}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::ros_callback( const sensor_msgs::LaserScan::ConstPtr& lidar_msg )
{
	if( connected_ )
	{
		uint16_t distances[271];
		uint8_t albedos[271];

		for( int i = 0; i < 271; ++i )
		{
			if( i >= 45 and i < 226 )
			{
				//Convert meters to millimeters
				distances[i] = (uint16_t) (lidar_msg->ranges[270 - i] * 1000);
			}
			else
			{
				distances[i] = 0;
			}
			albedos[i] = 0;
		}

		// Do this shit properly
		struct timespec t;
		clock_gettime( CLOCK_MONOTONIC_RAW, &t );
		createTrame( distances, albedos, reinterpret_cast<char*>(&write_buffer_.front()), 0, 0, t );

		socket_->async_send( boost::asio::buffer( write_buffer_ ),
							 boost::bind( &RosLidar::do_send, this,
										  boost::asio::placeholders::error,
										  boost::asio::placeholders::bytes_transferred() ) );
	}
}

//--------------------------------------------------------------------------------------------------
//
void
RosLidar::createTrame( uint16_t dist[271], uint8_t albedo[271], char trame[4096], uint64_t nbMesures,
			 uint64_t nbTelegrammes, struct timespec timeInit )
{
	char buffer[20];

	memset( trame, '\0', 2000 );

	strcat( trame, "sRA " );//commandType

	strcat( trame, "LMDscandata " );//command

	strcat( trame, "1 " );//versionNumber

	strcat( trame, "1 " );//deviceNumber

	strcat( trame, "000000 " );//SerialNumber

	strcat( trame,
			"0 0 " );//status 0 0->OK 0 1 -> error 0 2 -> pollutionWarning 0 4 pollutionError

	sprintf( buffer, "%x ", (uint32_t) nbTelegrammes );
	strcat( trame, buffer );//TelegramCounter

	sprintf( buffer, "%x ", (uint32_t) nbMesures );
	strcat( trame, buffer );//ScanCounter

	sprintf( buffer, "%x ", (uint32_t) elapsedMillis( timeInit ) );
	strcat( trame, buffer );//timeSinceStartInμsec

	sprintf( buffer, "%x ", (uint32_t) elapsedMillis( timeInit ) );
	strcat( trame, buffer );//timeInμsec

	strcat( trame, "0 0 " );//Status of digsitalInputs ??

	strcat( trame, "0 0 " );//Status of digsitalOutputs ??

	strcat( trame, "0 " );//Reserved

	sprintf( buffer, "%x ", 1500 );
	strcat( trame, buffer );//Scan Frequency (15Hz)

	sprintf( buffer, "%x ", 1500 );
	strcat( trame, buffer );//Measurement Frequency ??

	strcat( trame, "0 " );//No encoders

	//Encoder position && speed not present because no encoders.

	strcat( trame, "1 " );//Amount of 16bits channels

	strcat( trame, "DIST1 " );//Type de message : distances

	strcat( trame, "3F800000 " );//scaleFactor

	strcat( trame, "00000000 " );//offset

	sprintf( buffer, "%x ", -135 );
	strcat( trame, buffer );//startAngle

	sprintf( buffer, "%x ", 5000 );
	strcat( trame, buffer );//steps ???

	sprintf( buffer, "%x ", 271 );
	strcat( trame, buffer );//amountofData

	for( int i = 0; i < 271; i++ )
	{
		int locDist = 0;
		if( dist[i] > 3999 || dist[i] < 20 )
		{
			locDist = 0;
		}
		else
		{
			locDist = dist[i];
		}
		sprintf( buffer, "%x ", locDist );
		strcat( trame, buffer );//distances
	}

	strcat( trame, "1 " );//Amount of bits channels

	strcat( trame, "RSSI1 " );//Type de message : intensités lumineuses

	strcat( trame, "3F800000 " );//scaleFactor

	strcat( trame, "00000000 " );//offset

	sprintf( buffer, "%x ", -135 );
	strcat( trame, buffer );//startAngle

	sprintf( buffer, "%x ", 5000 );
	strcat( trame, buffer );//steps ???

	sprintf( buffer, "%x ", 271 );
	strcat( trame, buffer );//amountofData

	for( int i = 0; i < 271; i++ )
	{
		int locLum = 0;
		if( dist[i] > 3999 || dist[i] < 20 )
		{
			locLum = 0;
		}
		else
		{
			locLum = 100;
		}
		sprintf( buffer, "%x ", locLum );
		strcat( trame, buffer );//distances
	}

	strcat( trame, "0 " );//Position data : 0 -> no position data

	strcat( trame, "0 " );//Device name : 0 -> no name

	strcat( trame, "0 " );//Device comment : 0 -> no comment

	strcat( trame, "0 " );//Device time : 0 -> no time

	strcat( trame, "0 " );//Event info : 0 -> no Event
}

//--------------------------------------------------------------------------------------------------
//
long
RosLidar::elapsedMillis( struct timespec dateDepart )
{
	struct timespec NOW;
	long ecart;

	clock_gettime( CLOCK_MONOTONIC_RAW, &NOW );

	ecart = (((long) (NOW.tv_sec) - (long) (dateDepart.tv_sec)) * 1000L) +
			((long) (NOW.tv_nsec) - (long) (dateDepart.tv_nsec)) / 1000000L;
	return ecart;
}
