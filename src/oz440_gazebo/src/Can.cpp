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

#include "DriverSocket.hpp"
#include "Can.h"

#include <linux/can.h>

using namespace std::chrono;
using namespace std::chrono_literals;

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

Can::Can(int server_port, std::shared_ptr<Log> log_ptr)
        : stop_{ false }
        , log_ptr_ {log_ptr}
        , connect_thread_ { }
        , read_thread_ { }
        , server_port_ {server_port}
        , connected_ { false }
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ { }
        , sync_gps_ { gps_fix_sub_, gps_vel_sub_, 10 }
{ }

Can::~Can()
{
	disconnect();
	close(server_socket_desc_);
}

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

void Can::init()
{
	for (int i = 0; i<10;i++){
		clock_gettime(CLOCK_MONOTONIC_RAW, &(lastIMU[i]));
	}
	connect_thread_ = std::thread( &Can::connect, this );
	connect_thread_.detach();

	read_thread_ = std::thread( &Can::read_thread, this );
	read_thread_.detach();
}

//--------------------------------------------------------------------------------------------------

void Can::subscribe( ros::NodeHandle &node ) {

	// subscribe to gps topic
	gps_fix_sub_.subscribe( node, "/oz440/navsat/fix", 5 );
	gps_vel_sub_.subscribe( node, "/oz440/navsat/vel", 5 );
	sync_gps_.registerCallback( boost::bind( &Can::callback_gps, this, _1, _2 ) );

	// subscribe to imu topic
	imu_sub_ = node.subscribe( "/oz440/imu/data", 50, &Can::callback_imu, this );
}

//--------------------------------------------------------------------------------------------------

void Can::add_odo_packet( const std::array<bool, 4>& ticks )
{
	uint8_t data[ 1 ];

	data[ 0 ] = 0x00;

	if( ticks[ 0 ] )
	{
		data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 0 ) ) ) ;
	}

	if( ticks[ 1 ] )
	{
		data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 1 ) ) ) ;
	}

	if( ticks[ 2 ] )
	{
		data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 2 ) ) ) ;
	}

	if( ticks[ 3 ] )
	{
		data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 3 ) ) ) ;
	}

	send_packet( CanMessageId::CAN_ID_GEN, CanMessageType::CAN_MOT_CONS, data, 1 );
}

//--------------------------------------------------------------------------------------------------

void Can::cleanup()
{
	stop_ = true;
	disconnect();
	close(server_socket_desc_);
}

//--------------------------------------------------------------------------------------------------

bool Can::connected(){
	return connected_;
}

//--------------------------------------------------------------------------------------------------

void Can::connect(){

	server_socket_desc_ = DriverSocket::openSocketServer( (uint16_t)server_port_ );

	while( !stop_ )
	{
		if( !connected_ and server_socket_desc_ > 0 )
		{
			socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_ );

			if ( socket_desc_ > 0 )
			{
				connected_ = true;

				ROS_ERROR( "OzCore Can Socket Connected" );
			}
		}
		else{
			std::this_thread::sleep_for(500ms);
		}
	}
}

//--------------------------------------------------------------------------------------------------

void Can::read_thread()
{
	uint8_t tool_position_oz = 0;

	struct can_frame frame;

	memset( &frame, 0, sizeof( frame ) );
	while ( !stop_ )
	{
		//Mouvement automatisé de l'outil
		if ( move_requested_ )
		{
			if ( elapsedMicros( lastTool ) > 100000 )
			{
				clock_gettime(CLOCK_MONOTONIC_RAW, &lastTool );

				if ( tool_position_oz == target_position_ )
				{
					move_requested_ = false;
				}
				else
				{
					if ( tool_position_oz < target_position_ )
					{
						tool_position_oz++;
					}
					else
					{
						tool_position_oz--;
					}
				}
			}
		}
		
		if (connected_)
		{
			socket_access_.lock();
			ssize_t size = read( socket_desc_, &frame, sizeof( frame ));;
			socket_access_.unlock();

			if ( size > 0 )
			{
				if( ( ( frame.can_id ) >> 7 ) == CAN_ID_VER )
				{
					if ( frame.can_id % 16 == CAN_VER_CONS)
					{
						float actuator_order = frame.data[0];

						std::string to_log;

						std::time_t t = std::time( NULL );
						char buff[256];
						std::string format{ "%H:%M:%S" };

						size_t bytesRead = std::strftime( buff, 256, format.c_str(), std::localtime( &t ) );

						std::string date_str = std::string( buff, bytesRead );

						to_log.append( "[ " );
						to_log.append( date_str );
						to_log.append( " ]   " );
						to_log.append( "Tool_position : " );

						if (actuator_order == 1 )
						{
							if ( tool_position_oz != 0 )
							{
								tool_position_oz -= 1;
							}
							
							to_log.append( std::to_string(tool_position_oz) );
							log_ptr_-> write( to_log );
						}
						else if (actuator_order == 2 )
						{
							if ( tool_position_oz < 100 )
							{
								tool_position_oz += 1;
							}
							
							to_log.append( std::to_string(tool_position_oz) );
							log_ptr_-> write( to_log );
						}
						move_requested_ = false;
					}
					if ( frame.can_id % 16  == CAN_VER_PERCENT )
					{
						move_requested_ = true;
						target_position_ = frame.data[0];
						clock_gettime(CLOCK_MONOTONIC_RAW, &lastTool );
					}
				}
				else if ( frame.can_id >> 7 == CAN_ID_VER | ( CAN_RTR_FLAG >> 7 ) )
				{
					if( ( frame.can_id & 0x0F ) == CAN_VER_POS )
					{
						uint8_t data[1];

						data[ 0 ] = tool_position_oz;

						send_packet( CanMessageId::CAN_ID_VER, CanMessageType::CAN_VER_POS, data, 1 );
					}
				}
				else if ( frame.can_id >> 7 == CAN_ID_IHM )
				{
					if( frame.can_id  % 16 == CAN_IHM_BUZ )
					{
						std::string to_log;

						std::time_t t = std::time( NULL );
						char buff[256];
						std::string format{ "%H:%M:%S" };

						size_t bytesRead = std::strftime( buff, 256, format.c_str(), std::localtime( &t ) );

						std::string date_str = std::string( buff, bytesRead );

						to_log.append( "[ " );
						to_log.append( date_str );
						to_log.append( " ]   " );
						to_log.append( "Bip" );

						log_ptr_-> write( to_log );
					}
				}
			}
			else
			{
				if( errno == 32 or errno == 104 or ( size == 0 and errno == 11 )){
					disconnect();
				}
			}
			std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
		}
		else
		{
			std::this_thread::sleep_for(100ms);
		}
	}
}

//--------------------------------------------------------------------------------------------------

void Can::send_packet( CanMessageId id, CanMessageType id_msg, uint8_t data[], uint8_t len ){

	try {

		if (connected_) {

			struct can_frame frame;

			frame.can_id = (unsigned int) (id * 128 + id_msg);
			frame.can_dlc = len;

			std::memcpy( frame.data, data, len );

			socket_access_.lock();
			ssize_t nbytes = write( socket_desc_, &frame, sizeof(struct can_frame) );
			socket_access_.unlock();

			if (nbytes <= 0) {
				std::cout << "Can write error." << std::endl;
			}
		}
	}
	catch ( std::exception e ) {
		std::cout<<"Exception connect_can catch : "<< e.what() << std::endl;
	}
}

//--------------------------------------------------------------------------------------------------

void Can::disconnect(){

	close( socket_desc_ );
	connected_ = false;

	ROS_ERROR("OzCore Can Socket Disconnected");
}

//--------------------------------------------------------------------------------------------------

void
Can::callback_imu( const sensor_msgs::Imu::ConstPtr& imu_msg )
{
	if( connected_ )
	{
		long elapsedReal = elapsedMicros( lastIMU[currentIMU] );
		clock_gettime(CLOCK_MONOTONIC_RAW, &(lastIMU[currentIMU]));
		
		currentIMU++;
		if (currentIMU >= 10){
			currentIMU = 0;
		}
		
		double realTimeFactor = 100000. / elapsedReal;
		
		if (realTimeFactor > 1){
			realTimeFactor = 1;
		}
		
		uint8_t data[6];

		std::array<int16_t, 3> gyro_packet;

		gyro_packet.at(0) = static_cast<int16_t>( imu_msg->angular_velocity.x * realTimeFactor * 1000.0 * 360.0 /
		                                          ( 2.0 * M_PI * -30.5 ) );
		gyro_packet.at(1) = static_cast<int16_t>( imu_msg->angular_velocity.y * realTimeFactor * 1000.0 * 360.0 /
		                                          ( 2.0 * M_PI * -30.5 ) );
		gyro_packet.at(2) = static_cast<int16_t>( imu_msg->angular_velocity.z * realTimeFactor * 1000.0 * 360.0 /
		                                          ( 2.0 * M_PI * -30.5 ) );

		data[0] = (uint8_t) ((gyro_packet.at(0) >> 8) & 0xFF);
		data[1] = (uint8_t) ((gyro_packet.at(0) >> 0) & 0xFF);

		data[2] = (uint8_t) ((gyro_packet.at(1) >> 8) & 0xFF);
		data[3] = (uint8_t) ((gyro_packet.at(1) >> 0) & 0xFF);

		data[4] = (uint8_t) ((gyro_packet.at(2) >> 8) & 0xFF);
		data[5] = (uint8_t) ((gyro_packet.at(2) >> 0) & 0xFF);

		send_packet(CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_GYRO, data, 6);

		ROS_INFO( "Gyro packet enqueued" );

		std::array<int16_t, 3> accelero_packet;

		accelero_packet.at(0) = static_cast<int16_t>(imu_msg->linear_acceleration.x * realTimeFactor * 1000.0 / 9.8);
		accelero_packet.at(1) = static_cast<int16_t>(imu_msg->linear_acceleration.y * realTimeFactor * 1000.0 / 9.8);
		accelero_packet.at(2) = static_cast<int16_t>(imu_msg->linear_acceleration.z * realTimeFactor * -1000.0 / 9.8);

		data[0] = (uint8_t) ((accelero_packet.at(0) >> 8) & 0xFF);
		data[1] = (uint8_t) ((accelero_packet.at(0) >> 0) & 0xFF);

		data[2] = (uint8_t) ((accelero_packet.at(1) >> 8) & 0xFF);
		data[3] = (uint8_t) ((accelero_packet.at(1) >> 0) & 0xFF);

		data[4] = (uint8_t) ((accelero_packet.at(2) >> 8) & 0xFF);
		data[5] = (uint8_t) ((accelero_packet.at(2) >> 0) & 0xFF);

		send_packet(CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_ACC, data, 6);

		ROS_INFO( "Accelero packet enqueued" );
	}
}

long Can::elapsedMicros(struct timespec dateDepart){
	struct timespec NOW;
	long ecart;

	clock_gettime(CLOCK_MONOTONIC_RAW, &NOW);

	ecart = (((long)(NOW.tv_sec) - (long)(dateDepart.tv_sec)) * 1000000L) + ((long)(NOW.tv_nsec) - (long)(dateDepart.tv_nsec))/1000L;
	return ecart;
}

//--------------------------------------------------------------------------------------------------

void Can::callback_gps( const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg,
						const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg ) {
	if (connected_)
	{
		std::array<std::string, 3> frames = gps_.make_frame( gps_fix_msg->latitude,
		                                                     gps_fix_msg->longitude,
		                                                     gps_fix_msg->altitude,
		                                                     3, 3,
		                                                     sqrt( gps_vel_msg->vector.x * gps_vel_msg->vector.x +
		                                                           gps_vel_msg->vector.y * gps_vel_msg->vector.y ) );
		for (const auto &frame : frames) {
			if (!frame.empty()) {
				send_gps_frame(frame);
			}

			ROS_INFO("Gps packets sent");
		}
	}
}

//--------------------------------------------------------------------------------------------------

void Can::send_gps_frame( std::string frame )
{
	uint8_t data[ 1 ];
	uint8_t end_data[ 1 ];
	end_data[ 0 ] = 10;

	for( int i = 0 ; i < frame.size() ; i++ )
	{
		data[ 0 ] = static_cast<uint8_t>( frame.at( (uint8_t) i ) );
		send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, data, 1 );

		usleep( 200 );
	}

	send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, end_data, 1 );

	usleep( 200 );
}

//--------------------------------------------------------------------------------------------------
