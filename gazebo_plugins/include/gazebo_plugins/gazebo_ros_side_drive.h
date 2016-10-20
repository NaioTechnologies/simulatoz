
/*
 * \file  gazebo_ros_side_drive.h
 *
 * \brief differential drive with four wheels
 *
 * \author  Fanny Risbourg
 *
 */

#ifndef GAZEBO_ROS_SIDE_DRIVE_H_
#define GAZEBO_ROS_SIDE_DRIVE_H_

#include "gazebo/common/common.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

// ROS
#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"

// Boost
#include <boost/thread.hpp>

namespace gazebo {

    class Joint;
    class Entity;

    class GazeboRosSideDrive : public ModelPlugin {

    public:
        GazeboRosSideDrive();
        ~GazeboRosSideDrive();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate();

    private:

        physics::WorldPtr world;
        physics::ModelPtr parent;
        event::ConnectionPtr updateConnection;

        std::string left_front_joint_name_;
        std::string right_front_joint_name_;
        std::string left_rear_joint_name_;
        std::string right_rear_joint_name_;

        double wheel_diameter_;
        double torque;
        double wheel_speed_[4];

        physics::JointPtr joints[4];

        // ROS STUFF
        ros::NodeHandle* rosnode_;
        ros::Subscriber cmd_vel_subscriber_;

        std::string robot_namespace_;
        std::string command_topic_;

        // DiffDrive stuff
        void cmdVelCallback(const geometry_msgs::Vector3::ConstPtr& cmd_msg);

        double x_;
        double y_;


    };

}


#endif /* GAZEBO_ROS_SIDE_DRIVE_H_ */
