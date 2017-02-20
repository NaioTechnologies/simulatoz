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

#ifndef PROJECT_LOG_H
#define PROJECT_LOG_H

//==================================================================================================
// I N C L U D E   F I L E S

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

#include "ros/ros.h"

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class Log {
//-- Methods ---------------------------------------------------------------------------------------
public:
    Log( std::string log_folder );
    ~Log();

    void write( std::string text );

    void cleanup();

private:
    bool setup_log_folder( std::string log_folder_);
    bool create_file ();


//-- Data members ----------------------------------------------------------------------------------
private:
    std::string folder_;
    std::string filename_;
};

#endif //PROJECT_LOG_H
