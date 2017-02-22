
#include "DriverSocket.hpp"
#include "Can.h"

#include <linux/can.h>

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Can::Can(int server_port)
        : stop_asked_{ false }
        , connect_thread_ { }
        , read_thread_ { }
        , server_port_ {server_port}
        , socket_connected_ { false }
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ { }
        , tool_position_access_ { }
        , tool_position_ { 0 }
        , gps_packet_access_ { }
        , gps_packet_ { }
        , gps_manager_thread_ { }
        , last_gps_packet_ { }

{
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
}

//*****************************************  --  ADD PACKET  --  *******************************************************

void Can::add_actuator_position( uint8_t actuator_position )
{

    tool_position_access_.lock();
    tool_position_ = actuator_position;
    tool_position_access_.unlock();

}

//*****************************************  --  ADD PACKET  --  *******************************************************

void Can::add_gps_packet( Gps_packet gps_packet )
{
    gps_packet_access_.lock();
    last_gps_packet_ = gps_packet_;
    gps_packet_ = gps_packet;
    gps_packet_access_.unlock();

    gps_manager();
}


//*****************************************  --  ADD PACKET  --  *******************************************************

void Can::add_gyro_packet( std::array<int16_t, 3> gyro_packet )
{
    uint8_t data[6];

    data[0] = (uint8_t) ((gyro_packet.at(0) >> 8) & 0xFF);
    data[1] = (uint8_t) ((gyro_packet.at(0) >> 0) & 0xFF);

    data[2] = (uint8_t) ((gyro_packet.at(1) >> 8) & 0xFF);
    data[3] = (uint8_t) ((gyro_packet.at(1) >> 0) & 0xFF);

    data[4] = (uint8_t) ((gyro_packet.at(2) >> 8) & 0xFF);
    data[5] = (uint8_t) ((gyro_packet.at(2) >> 0) & 0xFF);

    send_packet(CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_GYRO, data, 6);
}

//*****************************************  --  ADD PACKET  --  *******************************************************

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

//*****************************************  --  ADD PACKET  --  *******************************************************

void Can::add_accelero_packet( std::array<int16_t, 3> accelero_packet )
{
    uint8_t data[6];

    data[0] = (uint8_t) ((accelero_packet.at(0) >> 8) & 0xFF);
    data[1] = (uint8_t) ((accelero_packet.at(0) >> 0) & 0xFF);

    data[2] = (uint8_t) ((accelero_packet.at(1) >> 8) & 0xFF);
    data[3] = (uint8_t) ((accelero_packet.at(1) >> 0) & 0xFF);

    data[4] = (uint8_t) ((accelero_packet.at(2) >> 8) & 0xFF);
    data[5] = (uint8_t) ((accelero_packet.at(2) >> 0) & 0xFF);

    send_packet(CanMessageId::CAN_ID_IMU, CanMessageType::CAN_IMU_ACC, data, 6);
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
}

//*****************************************  --  READ THREAD  --  ******************************************************

void Can::read_thread(){

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
//                            actuator_order_ = frame_.data[ 0 ] ;
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

}

//*****************************************  --  SEND PACKET  --  ******************************************************

void Can::send_packet( CanMessageId id, CanMessageType id_msg, uint8_t data[], uint8_t len ){

    try {

        if (socket_connected_) {

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
        Gps_packet gps_packet = gps_packet_;
        Gps_packet last_gps_packet = last_gps_packet_;
        gps_packet_access_.unlock();

        if( ( gps_packet.updated ) and ( last_gps_packet.updated ))
        {
            // compute speed and track orientation
            double track_orientation = north_bearing( last_gps_packet.lat, last_gps_packet.lon, gps_packet.lat, gps_packet.lon );

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
            sprintf( gs, "%03.1f", gps_packet.groundSpeed );

            std::strftime( hhmmss, 80, "%H%M%S", timeinfo );
            std::strftime( ddmmyy, 80, "%d%m%y", timeinfo );

            GeoAngle ns = GeoAngle::from_double( gps_packet.lat );
            GeoAngle we = GeoAngle::from_double( gps_packet.lon );

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

            sprintf( quality, "%d", static_cast<int>( gps_packet.quality ) );
            sprintf( nos, "%02d", static_cast<int>( gps_packet.satUsed ) );
            sprintf( alt, "%03.1f", gps_packet.alt );

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
