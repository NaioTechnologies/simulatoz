
#include "../include/DriverSocket.hpp"
#include "../include/Can.h"

#include <linux/can.h>

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Can::Can(int server_port)
        : stop_asked_{ false }
        , connect_thread_started_ { false }
        , connect_thread_ { }
        , read_thread_started_ { false }
        , read_thread_ { }
        , manage_thread_started_ { false }
        , manage_thread_ { }
        , server_port_ {server_port}
        , socket_connected_ { false }
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ { }
        , packets_ptr_ { nullptr }
        , packets_access_ { }
        , last_odo_packet_ptr_ { nullptr }
        , tool_position_access_ { }
        , tool_position_ { 0 }
        , actuator_packet_access_ { }
        , actuator_packet_ptr_ { nullptr }
        , gps_packet_access_ { }
        , gps_packet_ptr_ { nullptr }
        , gps_manager_thread_ { }
        , last_gps_packet_ptr_ { nullptr }

{
    last_odo_ticks_[0] = false;
    last_odo_ticks_[1] = false;
    last_odo_ticks_[2] = false;
    last_odo_ticks_[3] = false;

    init();
}

Can::~Can()
{
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  INIT  --  *************************************************************

void Can::init()
{
    connect_thread_ = std::thread( &Can::connect, this );
    connect_thread_.detach();

    read_thread_ = std::thread( &Can::read_thread, this );
    read_thread_.detach();

    manage_thread_ = std::thread( &Can::manage_thread, this );
    manage_thread_.detach();
}

//*****************************************  --  ADD PACKET  --  *******************************************************

void Can::add_packet( BaseNaio01PacketPtr packet_ptr )
{
    packets_access_.lock();
    packets_ptr_.emplace( packets_ptr_.begin(), packet_ptr );
    packets_access_.unlock();
}

//*****************************************  --  GET ACTUATOR PACKET  --  *******************************************************

ApiMoveActuatorPacketPtr Can::get_actuator_packet_ptr()
{
    actuator_packet_access_.lock();
    ApiMoveActuatorPacketPtr actuator_packet_ptr = actuator_packet_ptr_;
    actuator_packet_access_.unlock();

    return actuator_packet_ptr;
}
//*****************************************  --  ASK STOP  --  *********************************************************

void Can::ask_stop()
{
    stop_asked_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECTED?  --  *******************************************************

bool Can::connected(){
    return socket_connected_;
}

//*****************************************  --  CONNECT  --  **********************************************************

void Can::connect(){

    connect_thread_started_ = true;

    server_socket_desc_ = DriverSocket::openSocketServer( (uint16_t)server_port_ );

    while( !stop_asked_ )
    {
        if( !socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_asked_ );

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

    connect_thread_started_ = false;
}

//*****************************************  --  READ THREAD  --  ******************************************************

void Can::read_thread(){

    read_thread_started_ = true;

    struct can_frame frame;

    memset( &frame, 0, sizeof( frame ) );while ( !stop_asked_ )

    while ( !stop_asked_ )
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
                    if( ( ( frame.can_id ) % 16 ) == CAN_VER_CONS )
                    {
                        actuator_packet_access_.lock();
                        actuator_packet_ptr_ = std::make_shared<ApiMoveActuatorPacket>( frame.data[ 0 ] );
                        actuator_packet_access_.unlock();
                    }
                    else if( ( ( frame.can_id ) % 16 ) == CAN_VER_POS )
                    {
                        uint8_t data[1];

                        tool_position_access_.lock();
                        data[ 0 ] = tool_position_;
                        tool_position_access_.unlock();

                        send_packet( CanMessageId::CAN_ID_VER, CanMessageType::CAN_VER_POS, data, 1 );
                    }
                }
//                else if( ( ( frame.can_id ) >> 7 ) == CAN_ID_TELECO )
//                {
//                    if( ( ( frame.can_id ) % 16 ) == CAN_TELECO_NUM_VERSION )
//                    {
//                        uint8_t remote_data[ 8 ];
//
//                        std::cout << "setting teleco act : " << static_cast<int>( frame.data[ 6 ] ) << " self_id : " << static_cast<int>(  frame.data[ 7 ] ) << std::endl;
//
//                        com_ozcore_remote_status_.teleco_self_id_6 = frame.data[ 7 ];
//
//                        remote_data[ 0 ] = 0x10;
//                        remote_data[ 1 ] = 0x08;
//                        remote_data[ 2 ] = 0x00;
//                        remote_data[ 3 ] = 0x00;
//                        remote_data[ 4 ] = 0x00;
//                        remote_data[ 5 ] = 0x00;
//                        remote_data[ 6 ] = com_ozcore_remote_status_.teleco_self_id_6;
//                        remote_data[ 7 ] = com_ozcore_remote_status_.teleco_act_7;
//
//                        if( com_ozcore_remote_status_.teleco_act_7 < 10 )
//                        {
//                            remote_data[ 2 ] = ( 4 + 128 );
//                        }
//                        else if( com_ozcore_remote_status_.teleco_act_7 > 10 )
//                        {
//                            remote_data[ 2 ] = ( 16 + 32 );
//                        }
//
//                        send_packet( CanMessageId::CAN_ID_TELECO, CanMessageType::CAN_TELECO_NUM_VERSION, remote_data, 8 );
//
//                    }
//                }
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

    read_thread_started_ = false;

}

//*****************************************  --  MANAGE THREAD  --  ****************************************************

void Can::manage_thread(){

    manage_thread_started_ = true;

    try{

        BaseNaio01PacketPtr packet_ptr;

        while( !stop_asked_ )
        {
            if(socket_connected_) {

                if ( !packets_ptr_.empty()) {

                    packets_access_.lock();

                    packet_ptr = packets_ptr_.back();
                    manage_packet(packet_ptr);
                    packets_ptr_.pop_back();

                    packets_access_.unlock();
                }
                else{
                    std::this_thread::sleep_for(1ms);
                }
            }
            else{

                packets_ptr_.clear();
                std::this_thread::sleep_for(50ms);
            }
        }
    }
    catch ( std::exception e )
    {
        std::cout<<"Exception server_read_thread catch : "<< e.what() << std::endl;
    }

    manage_thread_started_ = false;

}

//*****************************************  --  MANAGE PACKET  --  ****************************************************

void Can::manage_packet( BaseNaio01PacketPtr packet_ptr ) {

    try {

        if (std::dynamic_pointer_cast<HaGyroPacket>( packet_ptr ))
        {
            HaGyroPacketPtr gyro_packet_ptr = std::dynamic_pointer_cast<HaGyroPacket>( packet_ptr );

            uint8_t data[ 6 ];

            data[ 0 ] = (uint8_t) (( gyro_packet_ptr->x >> 8 ) & 0xFF ) ;
            data[ 1 ] = (uint8_t) (( gyro_packet_ptr->x >> 0 ) & 0xFF ) ;

            data[ 2 ] = (uint8_t) (( gyro_packet_ptr->y >> 8 ) & 0xFF ) ;
            data[ 3 ] = (uint8_t) (( gyro_packet_ptr->y >> 0 ) & 0xFF ) ;

            data[ 4 ] = (uint8_t) (( gyro_packet_ptr->z >> 8 ) & 0xFF ) ;
            data[ 5 ] = (uint8_t) (( gyro_packet_ptr->z >> 0 ) & 0xFF ) ;

            send_packet( CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_GYRO, data, 6 );
        }
        else if (std::dynamic_pointer_cast<HaAcceleroPacket>( packet_ptr ))
        {
            HaAcceleroPacketPtr accel_packet_ptr = std::dynamic_pointer_cast<HaAcceleroPacket>( packet_ptr );

            uint8_t data[ 6 ];

            data[ 0 ] = (uint8_t) (( accel_packet_ptr->x >> 8 ) & 0xFF ) ;
            data[ 1 ] = (uint8_t) (( accel_packet_ptr->x >> 0 ) & 0xFF ) ;

            data[ 2 ] = (uint8_t) (( accel_packet_ptr->y >> 8 ) & 0xFF ) ;
            data[ 3 ] = (uint8_t) (( accel_packet_ptr->y >> 0 ) & 0xFF ) ;

            data[ 4 ] = (uint8_t) (( accel_packet_ptr->z >> 8 ) & 0xFF ) ;
            data[ 5 ] = (uint8_t) (( accel_packet_ptr->z >> 0 ) & 0xFF ) ;

            send_packet( CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_ACC, data, 6 );
        }
        else if (std::dynamic_pointer_cast<HaOdoPacket>( packet_ptr ))
        {
            HaOdoPacketPtr odo_packet_ptr = std::dynamic_pointer_cast<HaOdoPacket>( packet_ptr );

            if( last_odo_packet_ptr_ != nullptr )
            {
                if( last_odo_packet_ptr_->fr != odo_packet_ptr->fr )
                {
                    last_odo_ticks_[ 0 ] = not last_odo_ticks_[ 0 ];
                }

                if( last_odo_packet_ptr_->fl != odo_packet_ptr->fl )
                {
                    last_odo_ticks_[ 1 ] = not last_odo_ticks_[ 1 ];
                }

                if( last_odo_packet_ptr_->rr != odo_packet_ptr->rr )
                {
                    last_odo_ticks_[ 2 ] = not last_odo_ticks_[ 2 ];
                }

                if( last_odo_packet_ptr_->rl != odo_packet_ptr->rl )
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

            last_odo_packet_ptr_ = odo_packet_ptr;
        }
        else if (std::dynamic_pointer_cast<HaGpsPacket>( packet_ptr ))
        {
            HaGpsPacketPtr gps_packet_ptr = std::dynamic_pointer_cast<HaGpsPacket>(packet_ptr);

            gps_packet_access_.lock();
            last_gps_packet_ptr_ = gps_packet_ptr_;
            gps_packet_ptr_ = gps_packet_ptr;
            gps_packet_access_.unlock();

            gps_manager();
        }
        else if (std::dynamic_pointer_cast<ApiMoveActuatorPacket>(packet_ptr))
        {
            ApiMoveActuatorPacketPtr api_move_actuator_packet_ptr = std::dynamic_pointer_cast<ApiMoveActuatorPacket>(
                    packet_ptr);

            tool_position_access_.lock();
            tool_position_ = api_move_actuator_packet_ptr->position;
            tool_position_access_.unlock();

            uint8_t data[1];

            tool_position_access_.lock();
            data[0] = tool_position_;
            tool_position_access_.unlock();

            send_packet( CanMessageId::CAN_ID_VER, CanMessageType::CAN_VER_POS, data, 1);
        }

    }
    catch ( std::exception e ) {
        std::cout<<"Exception main_thread catch : "<< e.what() << std::endl;
    }

}

//*****************************************  --  SEND PACKET  --  ******************************************************

void Can::send_packet( CanMessageId id, CanMessageType id_msg, uint8_t data[], uint8_t len ){

    try {

        if (socket_connected_) {

            struct can_frame frame;
            ssize_t nbytes = -1;
            int nbTests = 0;

            frame.can_id = (unsigned int) (id * 128 + id_msg);
            frame.can_dlc = len;

            for (uint8_t i = 0; i < len; i++) {
                frame.data[i] = data[i];
            }

            while (nbytes <= 0 && nbTests < 10) {

                socket_access_.lock();
                nbytes = write( socket_desc_, &frame, sizeof(struct can_frame) );
                socket_access_.unlock();

                nbTests++;
            }

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

//*****************************************  --  GPS THREAD  --  *******************************************************

// Sends GPS packets to OzCore

void Can::gps_manager()
{
    try{

        gps_packet_access_.lock();
        HaGpsPacketPtr gps_packet_ptr = gps_packet_ptr_;
        HaGpsPacketPtr last_gps_packet_ptr = last_gps_packet_ptr_;
        gps_packet_access_.unlock();

        if( ( gps_packet_ptr != nullptr ) and ( last_gps_packet_ptr != nullptr ))
        {
            // compute speed and track orientation
            double track_orientation = north_bearing( last_gps_packet_ptr->lat, last_gps_packet_ptr->lon, gps_packet_ptr->lat, gps_packet_ptr->lon );

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
            sprintf( gs, "%03.1f", gps_packet_ptr_->groundSpeed );

            std::strftime( hhmmss, 80, "%H%M%S", timeinfo );
            std::strftime( ddmmyy, 80, "%d%m%y", timeinfo );

            GeoAngle ns = GeoAngle::from_double( gps_packet_ptr->lat );
            GeoAngle we = GeoAngle::from_double( gps_packet_ptr->lon );

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

            sprintf( quality, "%d", static_cast<int>( gps_packet_ptr_->quality ) );
            sprintf( nos, "%02d", static_cast<int>( gps_packet_ptr_->satUsed ) );
            sprintf( alt, "%03.1f", gps_packet_ptr_->alt );

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
