//
// Created by fanny on 25/11/16.
//

#ifndef PROJECT_METRIC_H
#define PROJECT_METRIC_H

#include "ros/ros.h"

#include <iostream>
#include <mutex>
#include <thread>

#include <vector>

#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"

#include "Test.hpp"

class Metric
{

public:
    // Constructeur/destructeur
    Metric( int argc, char **argv, Test test );
    ~Metric();

    // Getters
    bool followed_trajectory( );
    bool pushed_object();

    // callback functions
    void link_states_callback( const gazebo_msgs::LinkStates::ConstPtr& link_states_msg );

private:
    // functions
    void run();
    bool is_fallen( int index_link );
    void belong_to_trajectory( float* pose );
    void make_trajectory();

    //threads
    void trajectory_thread_function();
    void collision_thread_function();

private:

    bool collision_thread_started_;
    std::thread collision_thread_;

    bool trajectory_thread_started_;
    std::thread trajectory_thread_;

    gazebo_msgs::LinkStates::ConstPtr& link_states_;
    std::mutex link_states_access_;

    bool object_down_;
    bool followed_trajectory_;

    std::vector<float> trajectory_;
    std::vector<float> center_turns_;

    Test test_;

};

#endif //PROJECT_METRIC_H
