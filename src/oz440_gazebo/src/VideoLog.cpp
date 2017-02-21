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

#include "VideoLog.h"

//==================================================================================================
// C O N S T A N T S   &   L O C A L   V A R I A B L E S

//==================================================================================================
// P I M P L   C O D E   S E T C T I O N

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

VideoLog::VideoLog( std::string video_log_folder )
        : top_camera_sub_ { }
        , video_log_folder_ { video_log_folder }
        , dated_folder_ { }

{ }

VideoLog::~VideoLog()
{ }

//*****************************************  --  SUBSCRIBE  --  ********************************************************

void
VideoLog::subscribe( image_transport::ImageTransport& it )
{
    // subscribe to top_camera topic
    top_camera_sub_ = it.subscribe( "/oz440/top_camera/image_raw", 5, &VideoLog::callback_top_camera, this );
}

// ***************************************** --  CAMERA CALLBACK  --***************************************************

void
VideoLog::callback_top_camera( const sensor_msgs::Image::ConstPtr& image )
{
    if( dated_folder_.empty() )
    {
        bool success = setup_video_folder();

        if( !success )
        {
            ROS_ERROR( "Error creating the video folder" );
        }
    }

    if( !output_video_.isOpened() )
    {
        std::string codec = "MJPG";
        cv::Size size( image->width, image->height );
        std::string filename = dated_folder_ + "/output.avi";

        output_video_.open( filename, CV_FOURCC( codec.c_str()[0], codec.c_str()[1], codec.c_str()[2],
                                                 codec.c_str()[3] ), 5, size, true );

        if( !output_video_.isOpened() )
        {
            ROS_ERROR(
                    "Could not create the output video! Check filename and/or support for codec." );
            exit( -1 );
        }
    }

}

// *********************************************************************************************************************

bool
VideoLog::setup_video_folder()
{
    namespace fs = boost::filesystem;
    bool success{ };

    // Check if folder exists.
    if( fs::exists( video_log_folder_ ) )
    {
        // We create a dated folder and two subfolders to write our images.
        std::time_t t = std::time( NULL );
        size_t bufferSize{ 256 };
        char buff[bufferSize];
        std::string format{ "%F_%H-%M" };
        size_t bytesRead = std::strftime( buff, bufferSize, format.c_str(), std::localtime( &t ) );

        std::string date_str = std::string( buff, bytesRead );
        std::replace( date_str.begin(), date_str.end(), ':', '_' );

        fs::path dated_folder_path{ video_log_folder_ };
        dated_folder_path /= ( date_str );

        // Creating the folders.
        if( !fs::exists( dated_folder_path ) )
        {
            fs::create_directory( dated_folder_path );
        }

        dated_folder_ = dated_folder_path.c_str();
        success = true;

        ROS_INFO( "video_folder_ %s", dated_folder_.c_str() );
    }
    else
    {
        ROS_ERROR( "video_log_folder_ does not exist" );
    }

    return success;
}