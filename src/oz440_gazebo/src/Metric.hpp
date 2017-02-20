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

#ifndef PROJECT_METRIC_H
#define PROJECT_METRIC_H

//==================================================================================================
// I N C L U D E   F I L E S

#include "ros/ros.h"

#include <atomic>
#include <mutex>
#include <thread>

#include <vector>

#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/LaserScan.h"

#include "Log.h"

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class Metric
{
//-- Methods ---------------------------------------------------------------------------------------

public:
    // Constructeur/destructeur
    Metric( std::shared_ptr<Log> log_ptr );
    ~Metric();

    //suscribe
    void subscribe (ros::NodeHandle& node);

    void cleanup();

private:
    // functions
    void init();

    void timer_thread();

    void log( std::string link_name);
    void log_fallen( std::string link_name);

    // callback functions
    void link_states_callback( const gazebo_msgs::LinkStates::ConstPtr& link_states_msg );

//-- Data members ----------------------------------------------------------------------------------

    std::vector< std::string > link_names_;
    std::vector< float > link_orientation_;
    std::vector< float > link_pose_;
    std::mutex link_states_access_;

    std::shared_ptr<Log> log_ptr_;
    ros::Subscriber link_states_sub_;

    std::thread timer_thread_;

    std::atomic< bool > do_log_;
    std::atomic< bool > stop_;

};

#endif //PROJECT_METRIC_H
