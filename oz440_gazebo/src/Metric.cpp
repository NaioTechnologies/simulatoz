//
// Created by fanny on 25/11/16.
//

#include "Metric.hpp"

#include <chrono>
#include <cmath>
#include <stdlib.h>

#include "std_msgs/Float64.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

using namespace std::chrono;


// *********************************************************************************************************************

Metric::Metric( int argc, char **argv, Test test )
{
    object_down_ = false;

    followed_trajectory_ = true;

    test_ = test;

    make_trajectory();

    run(argc, argv);

}

// *********************************************************************************************************************

Metric::~Metric(){}

// *********************************************************************************************************************

void Metric::run(int argc, char **argv) {

    ros::init( argc, argv, "core", ros::init_options::AnonymousName);

    //creates main thread
    main_thread_ = std::thread( &Metric::main_thread_function, this );

    // creates collision thread
    collision_thread_ = std::thread( &Metric::collision_thread_function, this );

    // creates trajectory thread
    trajectory_thread_ = std::thread( &Metric::trajectory_thread_function, this );

}

// *********************************************************************************************************************

void Metric::initialize(Test test)
{
    object_down_ = false;

    followed_trajectory_ = true;

    test_ = test;

    make_trajectory();

}

// *********************************************************************************************************************

bool Metric::followed_trajectory( ) {

    return(followed_trajectory_);

}

// *********************************************************************************************************************

bool Metric::pushed_object() {

    return(object_down_);

}


// *********************************************************************************************************************

bool Metric::is_arrived() {

    int number_rows = test_.get_number_rows();
    int length_rows = test_.get_length_rows();
    float width = test_.get_width();

    float goal_x = 1.0 + length_rows;

    if( number_rows % 2 == 0 ){
        goal_x =  1.0;

    }

    float goal_y = -(number_rows - 1) * width ;

    float distance = std::sqrt( std::pow((position_x_ - goal_x) , 2.0) + std::pow((position_y_ - goal_y) , 2.0));

    ROS_ERROR( "x = %f, y = %f, distance = %f", goal_x, goal_y, distance);

    if (distance < width)
    {
        return(true);
    }
    else
    {
        return(false);
    }

}

// *********************************************************************************************************************

void Metric::main_thread_function(){

    using namespace std::chrono_literals;

    main_thread_started_ = true;

    ros::NodeHandle n;

    // subscribe to link_states topic
    ros::Subscriber link_states_sub = n.subscribe( "/gazebo/link_states", 50000, &Metric::link_states_callback, this );

    while( ros::ok())
    {
        std::this_thread::sleep_for(250ms);

        ros::spinOnce();
    }

    main_thread_started_ = false;
}

// *********************************************************************************************************************

void Metric::collision_thread_function(){

    using namespace std::chrono_literals;

    collision_thread_started_ = true;

    std::this_thread::sleep_for(1000ms);

    while ( ros::ok())
    {
        if(!object_down_)
        {
            link_states_access_.lock();

            for(int i = 0; i < vegetable_orientation_.size() ; i++)
            {
                if (vegetable_orientation_[i] > 0.0001)
                {
                    ROS_INFO( "object_down" );

                    object_down_=1;
                }
            }

            link_states_access_.unlock();
        }

        std::this_thread::sleep_for(500ms);
    }

    collision_thread_started_ = false;
}

// *********************************************************************************************************************

void Metric::trajectory_thread_function(){

    using namespace std::chrono_literals;

    std::this_thread::sleep_for(1000ms);

    trajectory_thread_started_ = true;

    while ( ros::ok())
    {
        if(followed_trajectory_)
        {
            link_states_access_.lock();

            belong_to_trajectory(position_x_, position_y_, position_z_);

            link_states_access_.unlock();
        }

        std::this_thread::sleep_for(500ms);
    }

    trajectory_thread_started_ = false;
}

// *********************************************************************************************************************

void Metric::link_states_callback(const gazebo_msgs::LinkStates::ConstPtr &link_states_msg)
{
    std::string type_vegetable = test_.get_type_vegetable();

    link_states_access_.lock();

    vegetable_orientation_.clear();

    for(int i = 0; i < link_states_msg->name.size() ; i++)
    {

        std::size_t found = link_states_msg->name[i].find(type_vegetable);

        if (found!=std::string::npos)
        {
            vegetable_orientation_.push_back(link_states_msg->pose[i].orientation.x);
        }
        else
        {
            std::size_t found_oz = link_states_msg->name[i].find("footprint");

            if (found_oz != std::string::npos) {

                position_x_ = link_states_msg->pose[i].position.x;
                position_y_ = link_states_msg->pose[i].position.y;
                position_z_ = link_states_msg->pose[i].position.z;

            }
        }
    }

    link_states_access_.unlock();

}

// *********************************************************************************************************************

void Metric::belong_to_trajectory( float pose_x, float pose_y, float pose_z ) {

    int distance = 0;
    int distance_x = 0;
    int distance_y = 0;
    float width = test_.get_width();

    followed_trajectory_ = false;

    // On suit les lignes
    for (int i = 0; i <  trajectory_.size() / 3 ; i++)
    {
        distance = std::sqrt( std::pow((pose_x - trajectory_[3 * i]) , 2.0) + std::pow((pose_y - trajectory_[3 * i + 1]) , 2.0) + std::pow((pose_z - trajectory_[3 * i + 2]) , 2.0));

        if (distance < width / 2.0 )
        {
            followed_trajectory_ = true;
        }
    }

    // On ne dépasse pas dans les virages
    if (followed_trajectory_ == false)
    {
        for (int i = 0; i < center_turns_.size() / 3 ; i++)
        {
            distance_x = abs(pose_x - center_turns_[3 * i]);
            distance_y = abs(pose_y - center_turns_[3 * i + 1]);

            if (distance_x < 2.0 + width / 2.0 and distance_y < 1.0 )
            {
                followed_trajectory_ = true;

            }
        }
    }
}

// *********************************************************************************************************************

void Metric::make_trajectory(){

    int number_rows = test_.get_number_rows();
    int length_rows = test_.get_length_rows();
    float width = test_.get_width();

    trajectory_.clear();
    center_turns_.clear();

    for(int i = 0; i < number_rows ; i++)
    {
        for(int j = 0; j < (length_rows) * 5; j++)
        {
            trajectory_.push_back( 1.0 + 0.2 * j);
            trajectory_.push_back(- i * width );
            trajectory_.push_back(0);
        }

        //centre des tournants
        center_turns_.push_back(0.0);
        center_turns_.push_back(- i * width );
        center_turns_.push_back(0);

        center_turns_.push_back(length_rows + 2.0);
        center_turns_.push_back(- i * width );
        center_turns_.push_back(0);

    }

}