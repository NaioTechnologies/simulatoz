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

#ifndef PROJECT_GPS_H
#define PROJECT_GPS_H

//==================================================================================================
// I N C L U D E   F I L E S

#include "GeoAngle.hpp"

#include <mutex>

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class Gps {

public:

    struct Gps_packet {
        double lat;
        double lon;
        double alt;
        uint8_t satUsed;
        uint8_t quality;
        double groundSpeed;
        bool updated;
    };

//-- Methods ---------------------------------------------------------------------------------------

    Gps( );
    ~Gps();

    std::array< std::string, 3 > make_frame( double lat, double lon, double alt, uint8_t satUsed, uint8_t quality, double groundSpeed );

    std::string get_rmc();
    std::string get_vtg();
    std::string get_gga();

    double north_bearing( double lat1, double lon1, double lat2, double lon2 );

//-- Data members ----------------------------------------------------------------------------------
private:

    std::time_t rawtime_;
    std::tm* timeinfo_;

    char hhmmss_[ 80 ];
    char ddmmyy_[ 80 ];
    char to_[ 80 ];
    char gs_[ 80 ];

    double track_orientation_;

    GeoAngle ns_ ;
    GeoAngle we_ ;

    char quality_[ 80 ];
    char nos_[ 80 ];
    char alt_[ 80 ];

    std::mutex gps_packet_access_;
    Gps_packet gps_packet_;
    Gps_packet last_gps_packet_;
};

#endif //PROJECT_GPS_H
