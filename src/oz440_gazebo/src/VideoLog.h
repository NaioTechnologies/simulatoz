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

#ifndef PROJECT_VIDEOLOG_H
#define PROJECT_VIDEOLOG_H

//==================================================================================================
// I N C L U D E   F I L E S

#include <atomic>

#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include "sensor_msgs/Image.h"

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class VideoLog {

//-- Methods ---------------------------------------------------------------------------------------
public:
    VideoLog( std::string video_log_folder );
    ~VideoLog();

    void subscribe( image_transport::ImageTransport& it );

private:
    void callback_top_camera(const sensor_msgs::Image::ConstPtr& image );

    bool setup_video_folder();


//-- Data members ----------------------------------------------------------------------------------
private:
    image_transport::Subscriber top_camera_sub_;

    std::string video_log_folder_;
    std::string dated_folder_;

    cv::VideoWriter output_video_;
};
#endif //PROJECT_VIDEOLOG_H

