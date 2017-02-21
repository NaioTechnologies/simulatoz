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

#include "Log.h"

//==================================================================================================
// C O N S T A N T S   &   L O C A L   V A R I A B L E S

//==================================================================================================
// P I M P L   C O D E   S E T C T I O N

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

Log::Log( std::string log_folder )
        : folder_ { }
        , filename_ { "" }
{
    setup_log_folder( log_folder );
}

Log::~Log()
{ }

// *********************************************************************************************************************

void Log::write( std::string text )
{
    namespace fs = boost::filesystem;

    std::ofstream file;

    file.open( filename_.c_str(), std::ios::app  );

    if (file.is_open())
    {
        file << text ;
        file << "\n";
    }
    else
    {
        ROS_ERROR( "Unable to open file");
    }

    file.close();
}

// *********************************************************************************************************************

bool Log::setup_log_folder( std::string log_folder )
{
    namespace fs = boost::filesystem;
    bool success{ };

    // Check if folder exists.
    if( fs::exists( log_folder ) )
    {
        // We create a dated folder and two subfolders to write our images.
        std::time_t t = std::time( NULL );
        size_t bufferSize{ 256 };
        char buff[bufferSize];
        std::string format{ "%F_%H-%M" };
        size_t bytesRead = std::strftime( buff, bufferSize, format.c_str(), std::localtime( &t ) );

        std::string date_str = std::string( buff, bytesRead );
        std::replace( date_str.begin(), date_str.end(), ':', '_' );

        fs::path dated_folder_path{ log_folder };
        dated_folder_path /= ( date_str );

        // Creating the folders.
        if( !fs::exists( dated_folder_path ) )
        {
            fs::create_directory( dated_folder_path );
        }

        folder_ = dated_folder_path.c_str();
        success = true;

        create_file();

        ROS_INFO( "Log_folder_ %s", folder_.c_str() );
    }
    else
    {
        ROS_ERROR( "Log_folder_ %s does not exist", log_folder.c_str() );
    }

    return success;
}

// *********************************************************************************************************************

bool Log::create_file( )
{
    namespace fs = boost::filesystem;
    bool success{ };

    // Check if folder exists.
    if( fs::exists( folder_ ) )
    {
        filename_ = folder_;
        filename_.append("/Log_metrics");

        // We create a file called Log_metric

        std::ofstream file;

        file.open ( filename_.c_str(), std::ios::app  );

        success = true;

        write( filename_ );

        ROS_INFO( "File is %s", filename_.c_str() );
    }
    else
    {
        ROS_ERROR( "folder_ %s does not exist", folder_.c_str() );
    }

    return success;
}
