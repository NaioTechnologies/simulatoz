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

using namespace std::chrono;
using namespace std::chrono_literals;


//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

Can::Can(uint16_t server_port)
        : stop_{ false }
        , connect_thread_ { }
        , read_thread_ { }
        , odometry_thread_ { }
        , server_port_ {server_port}
        , socket_connected_ { false }
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ { }
        , actuator_position_sub_ { }
        , imu_sub_ { }
        , gps_fix_sub_ { }
        , gps_vel_sub_ { }
        , sync_gps_ { gps_fix_sub_, gps_vel_sub_, 10 }
        , last_odo_packet_ { }
        , tool_position_access_ { }
        , tool_position_ { 0 }
        , actuator_order_ { }
        , actuator_order_access_ { }
        , last_order_down_ { 0 }
        , gps_packet_access_ { }
        , gps_packet_ { }
        , last_gps_packet_ { }

{

    last_odo_ticks_[0] = false;
    last_odo_ticks_[1] = false;
    last_odo_ticks_[2] = false;
    last_odo_ticks_[3] = false;

    last_odo_packet_ = {{2, 0, 0, 0}};

    gps_packet_.updated = false;
    last_gps_packet_.updated = false;

    listener_ptr_ = std::make_shared< tf::TransformListener >( ros::Duration( 10 ) );

    init();

}

Can::~Can()
{
    disconnect();
    close(server_socket_desc_);
}

//--------------------------------------------------------------------------------------------------

void Can::init()
{
    connect_thread_ = std::thread( &Can::connect, this );
    connect_thread_.detach();

    read_thread_ = std::thread( &Can::read_thread, this );
    read_thread_.detach();

    odometry_thread_ = std::thread( &Can::odometry_thread, this );
    odometry_thread_.detach();
}

//--------------------------------------------------------------------------------------------------


void Can::subscribe(ros::NodeHandle& node)
{
    // subscribe to gps topic
    gps_fix_sub_.subscribe(node, "/oz440/navsat/fix", 5);
    gps_vel_sub_.subscribe(node, "/oz440/navsat/vel", 5 );
    sync_gps_.registerCallback( boost::bind( &Can::callback_gps, this, _1, _2 ) );

    // subscribe to actuator position
    actuator_position_sub_ = node.subscribe( "/oz440/joint_states", 500, &Can::callback_actuator_position, this );

    // subscribe to imu topic
    imu_sub_ = node.subscribe( "/oz440/imu/data", 50, &Can::callback_imu, this );

    // Advertise actuator commands
//    actuator_pub_ = node.advertise< geometry_msgs::Vector3 >( "/oz440/cmd_act", 10 );
}

//--------------------------------------------------------------------------------------------------

void Can::cleanup()
{
    stop_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECT  --  **********************************************************

void Can::connect(){

    server_socket_desc_ = DriverSocket::openSocketServer( server_port_ );

    while( !stop_ )
    {
        if( !socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_ );

            if ( socket_desc_ > 0 )
            {
                socket_connected_ = true;

                ROS_ERROR( "OzCore Can Socket Connected" );
            }
        }
        else{
            std::this_thread::sleep_for(500ms);
        }
    }
}

//*****************************************  --  READ THREAD  --  ******************************************************

void Can::read_thread(){

    struct can_frame frame;

    memset( &frame, 0, sizeof( frame ) );

    while ( !stop_ )
    {
        if (socket_connected_)
        {
            socket_access_.lock();
            ssize_t size = read( socket_desc_, &frame, sizeof( frame ));;
            socket_access_.unlock();

            if ( size > 0 )
            {
                if( ( ( frame.can_id ) >> 7 ) == CAN_ID_VER )
                {
                    if (((frame.can_id) % 16) == CAN_VER_CONS)
                    {
                        actuator_order_ = frame.data[0];

//                        geometry_msgs::Vector3 command;
//
//                        if ( actuator_order_ == 1 )
//                        {
//                            command.x = actuator_position_ +0.005;
//                            ROS_INFO("MONTE : %f", actuator_position_ +0.005);
//                            last_order_down_ = 0;
//
//                        }
//                        else if ( actuator_order_ == 2 )
//                        {
//                            command.x = actuator_position_ -0.005;
//                            ROS_INFO("DESCEND : %f", actuator_position_ -0.005);
//                            last_order_down_ = 1;
//                        }
//                        else
//                        {
//                            if (last_order_down_ == 1) {
//                                command.x = actuator_position_ - 0.0001;
//                                std::this_thread::sleep_for(10ms);
//                            }
//                            else
//                            {
//                                command.x = actuator_position_ + 0.0001;
//                                std::this_thread::sleep_for(10ms);
//                            }
//                        }
//                        actuator_pub_.publish(command);

                    }
                    else if (((frame.can_id) % 16) == CAN_VER_POS) {
                        uint8_t data[1];

                        tool_position_access_.lock();
                        data[0] = tool_position_;
                        tool_position_access_.unlock();

                        send_packet(CanMessageId::CAN_ID_VER, CanMessageType::CAN_VER_POS, data, 1);
                    }

//                    else if ( ( (frame.can_id) >> 7 ) == CAN_ID_TELECO )
//                    {
//                        if ( ( (frame.can_id) % 16 ) == CAN_TELECO_NUM_VERSION )
//                        {
//                            uint8_t remote_data[8];
//
//                            std::cout << "setting teleco act : " << static_cast<int>( frame.data[6] ) << " self_id : "
//                                      << static_cast<int>(  frame.data[7] ) << std::endl;
//
//                            com_ozcore_remote_status_.teleco_self_id_6 = frame.data[7];
//
//                            remote_data[0] = 0x10;
//                            remote_data[1] = 0x08;
//                            remote_data[2] = 0x00;
//                            remote_data[3] = 0x00;
//                            remote_data[4] = 0x00;
//                            remote_data[5] = 0x00;
//                            remote_data[6] = com_ozcore_remote_status_.teleco_self_id_6;
//                            remote_data[7] = com_ozcore_remote_status_.teleco_act_7;
//
//                            if (com_ozcore_remote_status_.teleco_act_7 < 10) {
//                                remote_data[2] = (4 + 128);
//                            } else if (com_ozcore_remote_status_.teleco_act_7 > 10) {
//                                remote_data[2] = (16 + 32);
//                            }
//
//                            send_packet(CanMessageId::CAN_ID_TELECO, CanMessageType::CAN_TELECO_NUM_VERSION, remote_data, 8);
//                        }
//                    }
                }
            }
            else
            {
                if( errno == 32 or errno == 104 or ( size == 0 and errno == 11 )){
                    disconnect();
                }
            }
            std::this_thread::sleep_for( 1ms );
        }
        else
        {
            std::this_thread::sleep_for(100ms);
        }
    }
}

//*****************************************  --  SEND PACKET  --  ******************************************************

void Can::send_packet( CanMessageId id, CanMessageType id_msg, uint8_t data[], uint8_t len )
{
    try {

        if (socket_connected_) {

            struct can_frame frame;
            ssize_t nbytes;

            frame.can_id = (unsigned int) (id * 128 + id_msg);
            frame.can_dlc = len;

            std::memcpy( frame.data, data, len );

            socket_access_.lock();
            nbytes = write( socket_desc_, &frame, sizeof(struct can_frame) );
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

//*****************************************  --  DISCONNECT  --  *******************************************************

void Can::disconnect(){

    close( socket_desc_ );

    socket_connected_ = false;

    ROS_ERROR("OzCore Can Socket Disconnected");
}

// *********************************************************************************************************************

void Can::callback_gps( const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg,
                        const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg )
{
    if ( socket_connected_ ) {

        Gps_packet gps_packet;

        gps_packet.lat = gps_fix_msg->latitude;
        gps_packet.lon = gps_fix_msg->longitude;
        gps_packet.alt = gps_fix_msg->altitude;
        gps_packet.satUsed = 3;
        gps_packet.quality = 3;
        gps_packet.groundSpeed = sqrt(gps_vel_msg->vector.x * gps_vel_msg->vector.x +
                                      gps_vel_msg->vector.y * gps_vel_msg->vector.y);
        gps_packet.updated = true;

        gps_packet_access_.lock();
        last_gps_packet_ = gps_packet_;
        gps_packet_ = gps_packet;
        gps_packet_access_.unlock();

        gps_manager();

        ROS_INFO("Gps packet sent");
    }
}

// *********************************************************************************************************************

void
Can::callback_imu( const sensor_msgs::Imu::ConstPtr& imu_msg )
{
    if ( socket_connected_ ) {

        uint8_t data[6];

        std::array<int16_t, 3> gyro_packet;

        gyro_packet.at(0) = static_cast<int16_t>(imu_msg->angular_velocity.x * 1000.0 * 360.0 / (2.0 * M_PI * -30.5));
        gyro_packet.at(1) = static_cast<int16_t>(imu_msg->angular_velocity.y * 1000.0 * 360.0 / (2.0 * M_PI * -30.5));
        gyro_packet.at(2) = static_cast<int16_t>(imu_msg->angular_velocity.z * 1000.0 * 360.0 / (2.0 * M_PI * -30.5));

        data[0] = (uint8_t) ((gyro_packet.at(0) >> 8) & 0xFF);
        data[1] = (uint8_t) ((gyro_packet.at(0) >> 0) & 0xFF);

        data[2] = (uint8_t) ((gyro_packet.at(1) >> 8) & 0xFF);
        data[3] = (uint8_t) ((gyro_packet.at(1) >> 0) & 0xFF);

        data[4] = (uint8_t) ((gyro_packet.at(2) >> 8) & 0xFF);
        data[5] = (uint8_t) ((gyro_packet.at(2) >> 0) & 0xFF);

        send_packet(CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_GYRO, data, 6);

        ROS_INFO("Gyro packet sent");

        std::array<int16_t, 3> accelero_packet;

        accelero_packet.at(0) = static_cast<int16_t>(imu_msg->linear_acceleration.x * 1000.0 / 9.8);
        accelero_packet.at(1) = static_cast<int16_t>(imu_msg->linear_acceleration.y * 1000.0 / 9.8);
        accelero_packet.at(2) = static_cast<int16_t>(imu_msg->linear_acceleration.z * -1000.0 / 9.8);

        data[0] = (uint8_t) ((accelero_packet.at(0) >> 8) & 0xFF);
        data[1] = (uint8_t) ((accelero_packet.at(0) >> 0) & 0xFF);

        data[2] = (uint8_t) ((accelero_packet.at(1) >> 8) & 0xFF);
        data[3] = (uint8_t) ((accelero_packet.at(1) >> 0) & 0xFF);

        data[4] = (uint8_t) ((accelero_packet.at(2) >> 8) & 0xFF);
        data[5] = (uint8_t) ((accelero_packet.at(2) >> 0) & 0xFF);

        send_packet(CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_ACC, data, 6);

        ROS_INFO("Accelero packet sent");
    }
}

// *********************************************************************************************************************

void
Can::callback_actuator_position( const sensor_msgs::JointState::ConstPtr& joint_states_msg )
{
    if ( socket_connected_ ) {

        uint8_t position_percent = (uint8_t) (std::round(joint_states_msg->position[0] * (-100.0 / 0.15)));

        tool_position_access_.lock();
        tool_position_ = position_percent;
        tool_position_access_.unlock();

        ROS_INFO("Actuator position packet enqueued");
    }
}

//*****************************************  --  GPS MANAGER  --  *******************************************************

// Sends GPS packets to OzCore

void Can::gps_manager()
{
    try{

        gps_packet_access_.lock();
        Gps_packet gps_packet = gps_packet_;
        Gps_packet last_gps_packet = last_gps_packet_;
        gps_packet_access_.unlock();

        if( ( gps_packet.updated ) and ( last_gps_packet.updated ))
        {
            // compute speed and track orientation
            double track_orientation = north_bearing( last_gps_packet_.lat, last_gps_packet_.lon, gps_packet_.lat, gps_packet_.lon );

//          ********************************  RMC ********************************

            std::time_t rawtime;
            std::tm* timeinfo;

            char hhmmss[ 80 ];
            char ddmmyy[ 80 ];
            char to[ 80 ];
            char gs[ 80 ];

            std::time( &rawtime );
            timeinfo = std::localtime( &rawtime );

            sprintf( to, "%03.1f", track_orientation );
            sprintf( gs, "%03.1f", gps_packet_.groundSpeed );

            std::strftime( hhmmss, 80, "%H%M%S", timeinfo );
            std::strftime( ddmmyy, 80, "%d%m%y", timeinfo );

            GeoAngle ns = GeoAngle::from_double( gps_packet_.lat );
            GeoAngle we = GeoAngle::from_double( gps_packet_.lon );

            std::string gprmc =   std::string( "$GPRMC" ) + std::string( "," )
                                  + std::string( hhmmss ) + std::string( "," )
                                  + std::string( "A" ) + std::string( "," )
                                  + ns.to_string( true ) + std::string( "," )
                                  + we.to_string( false ) + std::string( "," )
                                  + std::string( gs ) + std::string( "," )
                                  + std::string( to ) + std::string( "," )
                                  + std::string( ddmmyy ) + std::string( "," )
                                  + std::string( "#" ) + std::string( "," )
                                  + std::string( "*" );

//          ********************************  VTG  ********************************

            std::string gpvtg =   std::string( "$GPVTG" ) + std::string( "," )
                                  + std::string( to ) + std::string( ",T," )
                                  + std::string( to ) + std::string( ",M," )
                                  + std::string( gs ) + std::string( ",N," )
                                  + std::string( gs ) + std::string( ",K," )
                                  + std::string( "*" );


//          ********************************  GGA  ********************************

            char quality[ 80 ];
            char nos[ 80 ];
            char alt[ 80 ];

            sprintf( quality, "%d", static_cast<int>( gps_packet_.quality ) );
            sprintf( nos, "%02d", static_cast<int>( gps_packet_.satUsed ) );
            sprintf( alt, "%03.1f", gps_packet_.alt );

            std::string gpgga =   std::string( "$GPGGA" ) + std::string( "," )
                                  + std::string( "#" ) + std::string( "," )
                                  + std::string( "#" ) + std::string( "," )
                                  + std::string( "#" ) + std::string( "," )
                                  + std::string( "#" ) + std::string( "," )
                                  + std::string( "#" ) + std::string( "," )
                                  + std::string( quality ) + std::string( "," )
                                  + std::string( nos ) + std::string( "," )
                                  + std::string( "0.9" ) + std::string( "," )
                                  + std::string( alt ) + std::string( ",M," )
                                  + std::string( "*" );

//          ********************************  SEND GPRMC  ********************************

            for( int i = 0 ; i < gprmc.size() ; i++ )
            {
                uint8_t data[ 1 ];

                data[ 0 ] = static_cast<uint8_t>( gprmc.at( (uint8_t) i ) );

                send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, data, 1 );

                usleep( 200 );
            }

            uint8_t end_data[ 1 ];
            end_data[ 0 ] = 10;

            send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, end_data, 1 );

            usleep( 200 );

//          ********************************  SEND GPVTG  ********************************

            for( int i = 0 ; i < gpvtg.size() ; i++ )
            {
                uint8_t data[ 1 ];

                data[ 0 ] = static_cast<uint8_t>( gpvtg.at( (uint8_t) i ) );

                send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, data, 1 );

                usleep( 200 );
            }

            send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, end_data, 1 );

            usleep( 200 );


//            ********************************  SEND GPGGA  ********************************

            for( int i = 0 ; i < gpgga.size() ; i++ )
            {
                uint8_t data[ 1 ];

                data[ 0 ] = static_cast<uint8_t>( gpgga.at( (uint8_t) i ) );

                send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, data, 1 );

                usleep( 200 );
            }

            send_packet( CanMessageId::CAN_ID_GPS, CanMessageType::CAN_GPS_DATA, end_data, 1 );

            usleep( 20 );
        }

    }
    catch ( std::exception e ) {
        std::cout<<"Exception gps_manager catch : "<< e.what() << std::endl;
    }
}

//*****************************************  --  GET NORTH BEARING  --  ************************************************

double Can::north_bearing( double lat1, double lon1, double lat2, double lon2 )
{
    double startLat = lat1 * M_PI / 180.0;
    double startLon = lon1 * M_PI / 180.0;

    double endLat = lat2 * M_PI / 180.0;
    double endLon = lon2 * M_PI / 180.0;

    double dLong = endLon - startLon;

    double dPhi = std::log(std::tan(endLat / 2.0 + M_PI / 4.0) / std::tan(startLat / 2.0 + M_PI / 4.0));

    if (std::abs(dLong) > M_PI) {
        if (dLong > 0.0) {
            dLong = -(2.0 * M_PI - dLong);
        } else {
            dLong = (2.0 * M_PI - dLong);
        }
    }

    double brng = fmod((std::atan2(dLong, dPhi) / M_PI * 180.0) + 360.0, 360.0);

    return brng;
}

// *********************************************************************************************************************

void
Can::odometry_thread()
{
    using namespace std::chrono_literals;
    uint8_t fr = 0;
    uint8_t br = 0;
    uint8_t bl = 0;
    uint8_t fl = 0;

    std::this_thread::sleep_for( 3000ms );

    while( socket_connected_ )
    {
        if( socket_connected_ )
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

            while( socket_connected_ )
            {
                bool tic = false;

                while( not tic and socket_connected_ )
                {

                    std::this_thread::sleep_for( 10ms );

                    pitch_bl = getPitch( "/back_left_wheel" );
                    pitch_fl = getPitch( "/front_left_wheel" );
                    pitch_br = getPitch( "/back_right_wheel" );
                    pitch_fr = getPitch( "/front_right_wheel" );

                    tic = odo_wheel( bl, pitch_bl, pitch_last_tic_bl, forward_backward_bl );
                    tic = odo_wheel( fl, pitch_fl, pitch_last_tic_fl, forward_backward_fl ) or tic;
                    tic = odo_wheel( br, pitch_br, pitch_last_tic_br, forward_backward_br ) or tic;
                    tic = odo_wheel( fr, pitch_fr, pitch_last_tic_fr, forward_backward_fr ) or tic;
                }

                if( last_odo_packet_.at(0) != 2 )
                {
                    if( last_odo_packet_.at(0) != fr )
                    {
                        last_odo_ticks_[ 0 ] = not last_odo_ticks_[ 0 ];
                    }

                    if( last_odo_packet_.at(1) != fl )
                    {
                        last_odo_ticks_[ 1 ] = not last_odo_ticks_[ 1 ];
                    }

                    if( last_odo_packet_.at(2) != br )
                    {
                        last_odo_ticks_[ 2 ] = not last_odo_ticks_[ 2 ];
                    }

                    if( last_odo_packet_.at(3) != bl )
                    {
                        last_odo_ticks_[ 3 ] = not last_odo_ticks_[ 3 ];
                    }
                }

                uint8_t data[ 1 ];

                data[ 0 ] = 0x00;

                if( last_odo_ticks_[ 0 ] )
                {
                    data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 0 ) ) ) ;
                }

                if( last_odo_ticks_[ 1 ] )
                {
                    data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 1 ) ) ) ;
                }

                if( last_odo_ticks_[ 2 ] )
                {
                    data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 2 ) ) ) ;
                }

                if( last_odo_ticks_[ 3 ] )
                {
                    data[ 0 ] = (uint8_t) (( data[ 0 ] | ( 0x01 << 3 ) ) ) ;
                }

                send_packet( CanMessageId::CAN_ID_GEN, CanMessageType::CAN_MOT_CONS, data, 1 );

                last_odo_packet_ = {fr, fl, br, bl};
            }
        }
        else
        {
            fr = 0;
            br = 0;
            bl = 0;
            fl = 0;

            std::this_thread::sleep_for( 100ms );
        }
    }
}

// *********************************************************************************************************************

double
Can::getPitch( std::string wheel )
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
Can::odo_wheel( uint8_t& odo_wheel, double& pitch, double& pitch_last_tic, int& forward_backward )
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
        odo_wheel = (uint8_t) (odo_wheel + 1 % 2);
        forward_backward = 0;
    }

    // Si on a commencé par reculer
    if( forward_backward == -1 and
        (pitch - pitch_last_tic <= -angle_tic or pitch - pitch_last_tic >= 0) )
    {
        pitch_last_tic = pitch;
        tic = true;
        odo_wheel = (uint8_t) (odo_wheel + 1 % 2);
        forward_backward = 0;
    }

    return tic;
}