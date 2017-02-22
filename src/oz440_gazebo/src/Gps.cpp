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

#include "Gps.h"

#include "ros/ros.h"

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

Gps::Gps( )
        : gps_packet_access_ { }
        , gps_packet_ { }
        , last_gps_packet_ { }
{ }

Gps::~Gps(){ }

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

std::array< std::string, 3 > Gps::make_frame( double lat, double lon, double alt, uint8_t satUsed, uint8_t quality, double groundSpeed )
{
    struct Gps_packet gps_packet;
    std::array< std::string, 3 > frames = { "", "", "" };

    gps_packet.lat = lat;
    gps_packet.lon = lon;
    gps_packet.alt = alt;
    gps_packet.satUsed = satUsed;
    gps_packet.quality = quality;
    gps_packet.groundSpeed = groundSpeed;
    gps_packet.updated = true;

    gps_packet_access_.lock();
    last_gps_packet_ = gps_packet_;
    gps_packet_ = gps_packet;
    gps_packet_access_.unlock();

    if( ( gps_packet.updated ) and ( last_gps_packet_.updated ) ) {

        track_orientation_ = north_bearing(last_gps_packet_.lat, last_gps_packet_.lon, gps_packet.lat, gps_packet.lon);

        std::time(&rawtime_);
        timeinfo_ = std::localtime(&rawtime_);

        sprintf(to_, "%03.1f", track_orientation_);
        sprintf(gs_, "%03.1f", gps_packet.groundSpeed);

        std::strftime(hhmmss_, 80, "%H%M%S", timeinfo_);
        std::strftime(ddmmyy_, 80, "%d%m%y", timeinfo_);

        ns_ = GeoAngle::from_double(gps_packet.lat);
        we_ = GeoAngle::from_double(gps_packet.lon);

        sprintf(quality_, "%d", static_cast<int>( gps_packet.quality ));
        sprintf(nos_, "%02d", static_cast<int>( gps_packet.satUsed ));
        sprintf(alt_, "%03.1f", gps_packet.alt);

        frames[0] = get_rmc();
        frames[1] = get_vtg();
        frames[2] = get_gga();
    }

    return frames;
}

//--------------------------------------------------------------------------------------------------

std::string Gps::get_rmc()
{
    std::string gprmc =   std::string( "$GPRMC" ) + std::string( "," )
                          + std::string( hhmmss_ ) + std::string( "," )
                          + std::string( "A" ) + std::string( "," )
                          + ns_.to_string( true ) + std::string( "," )
                          + we_.to_string( false ) + std::string( "," )
                          + std::string( gs_ ) + std::string( "," )
                          + std::string( to_ ) + std::string( "," )
                          + std::string( ddmmyy_ ) + std::string( "," )
                          + std::string( "#" ) + std::string( "," )
                          + std::string( "*" );
    return gprmc;
}

//--------------------------------------------------------------------------------------------------

std::string Gps::get_vtg() {

    std::string gpvtg =   std::string( "$GPVTG" ) + std::string( "," )
                          + std::string( to_ ) + std::string( ",T," )
                          + std::string( to_ ) + std::string( ",M," )
                          + std::string( gs_ ) + std::string( ",N," )
                          + std::string( gs_ ) + std::string( ",K," )
                          + std::string( "*" );
    return gpvtg;
}

//--------------------------------------------------------------------------------------------------

std::string Gps::get_gga()
{
    std::string gpgga =   std::string( "$GPGGA" ) + std::string( "," )
                          + std::string( "#" ) + std::string( "," )
                          + std::string( "#" ) + std::string( "," )
                          + std::string( "#" ) + std::string( "," )
                          + std::string( "#" ) + std::string( "," )
                          + std::string( "#" ) + std::string( "," )
                          + std::string( quality_ ) + std::string( "," )
                          + std::string( nos_ ) + std::string( "," )
                          + std::string( "0.9" ) + std::string( "," )
                          + std::string( alt_ ) + std::string( ",M," )
                          + std::string( "*" );
    return gpgga;
}

//--------------------------------------------------------------------------------------------------

double Gps::north_bearing( double lat1, double lon1, double lat2, double lon2 )
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
