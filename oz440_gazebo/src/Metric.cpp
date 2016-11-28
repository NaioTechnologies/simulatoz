//
// Created by fanny on 25/11/16.
//

#include "Metric.hpp"

#include <chrono>
#include <stdlib.h>

#include "std_msgs/Float64.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

using namespace std::chrono;


// *********************************************************************************************************************

Metric::Metric( int argc, char **argv, Test test )
{
    ros::init( argc, argv, "metric");

    ros::NodeHandle n;

    object_down_ = false;

    followed_trajectory_ = true;

    test_ = test;

    make_trajectory();;

    run();

}

// *********************************************************************************************************************

Metric::~Metric(){}

// *********************************************************************************************************************

void Metric::run() {

    ros::NodeHandle n;

    // subscribe to link_states topic
    ros::Subscriber link_states_sub = n.subscribe( "/gazebo/link_states", 500000, &Metric::link_states_callback, this );

    // creates collision thread
    collision_thread_ = std::thread( &Metric::collision_thread_function, this );

    // creates trajectory thread
    trajectory_thread_ = std::thread( &Metric::trajectory_thread_function, this );

    while( ros::ok())
    {
        std::this_thread::sleep_for(250ms);

        ros::spinOnce();
    }

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

void Metric::collision_thread_function(){

    using namespace std::chrono_literals;
    std::string type_vegetable = test_.get_type_vegetable();

    collision_thread_started_ = true;

    while ( ros::ok() and !object_down_)
    {
//        link_states_access_.lock();
//
//        for(int i = 0; i < sizeof(link_states_.name)/sizeof(link_states_.name[0]) ; i++)
//        {
//            std::size_t found = link_states_.name[i].find(type_vegetable);
//
//            if (found!=std::string::npos)
//            {
//                is_fallen(i);
//            }
//        }
//
//        link_states_access_.unlock();

        std::this_thread::sleep_for(500ms);
    }

    collision_thread_started_ = false;
}

// *********************************************************************************************************************

void Metric::trajectory_thread_function(){

    using namespace std::chrono_literals;

    geometry_msgs::Pose pose;
    geometry_msgs::Point point_position;
    int position [3] ;

    trajectory_thread_started_ = true;

    while ( ros::ok() and followed_trajectory_)
    {
//        link_states_access_.lock();
//
//        for(int i = 0; i < sizeof(link_states_.name)/sizeof(link_states_.name[0]) ; i++)
//        {
//            std::size_t found = link_states_.name[i].find("footprint");
//
//            if (found!=std::string::npos)
//            {
//                pose = link_states_.pose[index];
//                point_position = pose.position;
//                position[0] = point_position.x;
//                position[1] = point_position.y;
//                position[2] = point_position.z;
//
//                followed_trajectory_ = belong_to_trajectory();
//            }
//
//        }
//
//        link_states_access_.unlock();
    }

    trajectory_thread_started_ =false;
}

// *********************************************************************************************************************

void Metric::link_states_callback(const gazebo_msgs::LinkStates::ConstPtr &link_states_msg) {

    link_states_access_.lock();

    link_states_ = link_states_msg;

    link_states_access_.unlock();

}

// *********************************************************************************************************************

bool Metric::is_fallen( int index_link ) {

    geometry_msgs::Pose pose = link_states.pose[index];

    if(pose.orientation.x > 0.0001)
    {
        object_down_ = 1;
    }
}

// *********************************************************************************************************************

bool Metric::belong_to_trajectory( float* pose ) {

    bool belong_to_trajectory = false;
    int distance = 0;
    int distance_x = 0;
    int distance_y = 0;
    float width = test_.get_width();

    // On suit les lignes
    for (int i = 0; i < size(trajectory_) / 3 ; i++)
    {
        distance = math.sqrt( math.pow((pose[0] - trajectory_[3 * i]) , 2.0) + math.pow((pose[1] - trajectory_[3 * i + 1]) , 2.0) + math.pow((pose[2] - trajectory_[3 * i + 2]) , 2.0));

        if (distance < width / 2.0 )
        {
            belong_to_trajectory = true;
        }
    }

    // On ne dépasse pas dans les virages
    if (belong_to_trajectory == false)
    {
        for (int i = 0; i < size(center_turns_) / 3 ; i++)
        {
            distance_x = abs(pose[0] - center_turns[3 * i]);
            distance_y = abs(pose[1] - center_turns[3 * i + 1]);

            if (distance_x < 2.0 + width / 2.0 and distance_y < 1.0 )
            {
                belong_to_trajectory = true;
            }
        }
    }

    return (belong_to_trajectory);

}

// *********************************************************************************************************************

void Metric::make_trajectory(){

    int number_rows = test_.get_number_rows();
    int length_rows = test_.get_length_rows();
    float width = test_.get_width();

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