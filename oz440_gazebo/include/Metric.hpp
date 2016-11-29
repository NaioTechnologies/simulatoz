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
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/LaserScan.h"

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

    //Setter
    void initialize(Test test);

    // callback functions
    void link_states_callback( const gazebo_msgs::LinkStates::ConstPtr& link_states_msg );

private:
    // functions
    void run(int argc, char **argv);
    void belong_to_trajectory( float pose_x, float pose_y, float pose_z );
    void make_trajectory();

    //threads
    void trajectory_thread_function();
    void collision_thread_function();
    void main_thread_function();


private:

    bool collision_thread_started_;
    std::thread collision_thread_;

    bool main_thread_started_;
    std::thread main_thread_;

    bool trajectory_thread_started_;
    std::thread trajectory_thread_;

//    std::vector<std::string> link_states_names_;
    std::vector<float> vegetable_orientation_;
    float position_x_;
    float position_y_;
    float position_z_;
    std::mutex link_states_access_;

    bool object_down_;
    bool followed_trajectory_;

    std::vector<float> trajectory_;
    std::vector<float> center_turns_;

    Test test_;

};

#endif //PROJECT_METRIC_H
