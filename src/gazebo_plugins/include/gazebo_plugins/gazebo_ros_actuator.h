
/*
 * \file  gazebo_ros_side_drive.h
 *
 * \brief differential drive with four wheels
 *
 * \author  Fanny Risbourg
 *
 */

#ifndef GAZEBO_ROS_ACTUATOR_H_
#define GAZEBO_ROS_ACTUATOR_H_

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

    class GazeboRosActuator : public ModelPlugin {

    public:
        GazeboRosActuator();
        ~GazeboRosActuator();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate();

    private:

        physics::WorldPtr world;
        physics::ModelPtr parent;
        event::ConnectionPtr updateConnection;

        std::string joint_name_;

        double torque;
        float position_;

        physics::JointPtr joints;

        // ROS STUFF
        ros::NodeHandle* rosnode_;
        ros::Subscriber cmd_subscriber_;

        std::string robot_namespace_;
        std::string command_topic_;

        int connection_;

        // DiffDrive stuff
        void cmdCallback(const geometry_msgs::Vector3::ConstPtr& cmd_msg);

    };

}


#endif /* GAZEBO_ROS_ACTUATOR_H_ */
