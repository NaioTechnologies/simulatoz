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

#include "Metric.hpp"

#include <chrono>
#include <cmath>
#include <stdlib.h>

#include "std_msgs/Float64.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

//==================================================================================================
// C O N S T A N T S   &   L O C A L   V A R I A B L E S

using namespace std::chrono;

//==================================================================================================
// P I M P L   C O D E   S E T C T I O N

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

Metric::Metric( std::shared_ptr<Log> log_ptr )
        : log_ptr_ { log_ptr }
        , link_names_ { }
        , link_orientation_ { }
        , link_pose_ { }
        , link_states_access_ { }
        , link_states_sub_ { }
        , timer_thread_ { }
        , do_log_ { false }
        , stop_ { false }
{
    init();
}

//--------------------------------------------------------------------------------------------------

Metric::~Metric(){}

//--------------------------------------------------------------------------------------------------

void Metric::init() {

    //creates timer thread
    timer_thread_ = std::thread( &Metric::timer_thread, this );
    timer_thread_.detach();
}

//--------------------------------------------------------------------------------------------------

void Metric::cleanup()
{
    stop_ = true ;
}
//--------------------------------------------------------------------------------------------------

void Metric::subscribe( ros::NodeHandle& node)
{
    // subscribe to link_states topic
    link_states_sub_ = node.subscribe( "/gazebo/link_states", 1, &Metric::link_states_callback, this );

}
//--------------------------------------------------------------------------------------------------

void Metric::timer_thread()
{
    while( !stop_ )
    {
        do_log_ = true;
        std::this_thread::sleep_for( 1000ms );
    }
}

//--------------------------------------------------------------------------------------------------

void Metric::link_states_callback(const gazebo_msgs::LinkStates::ConstPtr &link_states_msg)
{
    if( do_log_ )
    {
        do_log_ = false ;

        link_states_access_.lock();

        link_names_.clear();
        link_pose_.clear();
        link_orientation_.clear();


        for (int i = 0; i < link_states_msg->name.size(); i++) {
            link_names_.push_back(link_states_msg->name[i]);

            link_pose_.push_back(link_states_msg->pose[i].position.x);
            link_pose_.push_back(link_states_msg->pose[i].position.y);
            link_pose_.push_back(link_states_msg->pose[i].position.z);

            link_orientation_.push_back(link_states_msg->pose[i].orientation.x);
            link_orientation_.push_back(link_states_msg->pose[i].orientation.y);
            link_orientation_.push_back(link_states_msg->pose[i].orientation.z);
        }

        link_states_access_.unlock();

        log("footprint");
        log_fallen("Leek");
        log_fallen("Red_stick");
    }
}

//--------------------------------------------------------------------------------------------------

void Metric::log( std::string link_name){

    link_states_access_.lock();

    for( int i = 0; i < link_names_.size() ; i++ ) {

        std::size_t found = link_names_[i].find( link_name );

        if (found != std::string::npos) {

            std::string to_log = link_name;
            to_log.append( " : pose = [ " );
            to_log.append( std::to_string( link_pose_[ i*3 ] ) );
            to_log.append( ", " );
            to_log.append( std::to_string( link_pose_[ i*3 + 1 ]) );
            to_log.append( ", "  );
            to_log.append( std::to_string( link_pose_[ i*3 + 2 ] ) );
            to_log.append( "] , orientation : [ "  );
            to_log.append( std::to_string( link_orientation_[ i*3 ] ) );
            to_log.append( ", " );
            to_log.append( std::to_string( link_orientation_[ i*3 + 1 ] ) );
            to_log.append( ", "  );
            to_log.append( std::to_string( link_orientation_[ i*3 + 2 ] ) );
            to_log.append( "] " );

            log_ptr_-> write( to_log );

        }
    }

    link_states_access_.unlock();
}


//--------------------------------------------------------------------------------------------------

void Metric::log_fallen( std::string link_name){

    std::vector< float > state;

    link_states_access_.lock();

    for( int i = 0; i < link_names_.size() ; i++ ) {

        std::size_t found = link_names_[i].find( link_name );

        if (found != std::string::npos) {

            if ( link_orientation_[ i*3 ] > 0.0001)
            {
                ROS_INFO( "%s has fallen. ", link_name.c_str() );

                std::string to_log = link_name;
                to_log.append( " fallen : pose = [ " );
                to_log.append( std::to_string( link_pose_[ i*3 ] ) );
                to_log.append( ", " );
                to_log.append( std::to_string( link_pose_[ i*3 + 1 ]) );
                to_log.append( ", "  );
                to_log.append( std::to_string( link_pose_[ i*3 + 2 ] ) );
                to_log.append( "] , orientation : [ "  );
                to_log.append( std::to_string( link_orientation_[ i*3 ] ) );
                to_log.append( ", " );
                to_log.append( std::to_string( link_orientation_[ i*3 + 1 ] ) );
                to_log.append( ", "  );
                to_log.append( std::to_string( link_orientation_[ i*3 + 2 ] ) );
                to_log.append( "] " );

                log_ptr_-> write( to_log );
            }
        }
    }

    link_states_access_.unlock();
}

//--------------------------------------------------------------------------------------------------
