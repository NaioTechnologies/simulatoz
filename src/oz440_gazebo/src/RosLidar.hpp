//==================================================================================================
//
//  Copyright(c)  2016  Naïo Technologies
//
//  These coded instructions, statements, and computer programs contain unpublished proprietary
//  information written by Naio Technologies and are protected by copyright law. They may not be
//  disclosed to third parties or copied or duplicated in any form, in whole or in part, without
//  the prior written consent of Naio Technologies.
//
//==================================================================================================

#ifndef RosLidar_hpp
#define RosLidar_hpp

//==================================================================================================
// I N C L U D E   F I L E S

#include <boost/asio.hpp>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class RosLidar
{
//-- Methods ---------------------------------------------------------------------------------------
public:
	RosLidar( boost::asio::io_service& io_service, uint16_t port );

	~RosLidar();

	void subscribe( ros::NodeHandle& node );

	void cleanup();

private:
	void do_accept( const boost::system::error_code& ec );

	void do_receive( const boost::system::error_code& ec, std::size_t bytes_transferred );

	void ros_callback( const sensor_msgs::LaserScan::ConstPtr& lidar_msg );


	//void read_thread();

	//void send_packet();

//-- Data members ----------------------------------------------------------------------------------
private:
	const std::string laser_topic_;
	const uint32_t laser_queue_;
	ros::Subscriber ros_sub_;

	boost::asio::ip::tcp::acceptor acceptor_;
	boost::asio::ip::tcp::socket socket_;
	bool connected_;
};

//==================================================================================================
// T E M P L A T E S   C O D E   S E C T I O N

//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif
