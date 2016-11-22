/*
 * \file  gazebo_ros_side_drive.h
 *
 * \brief differential drive with four wheels
 *
 * \author  Fanny Risbourg
 *
 */
#include <gazebo_plugins/gazebo_ros_actuator.h>

#include "gazebo/gazebo.hh"

#include <sdf/sdf.hh>

#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"

#include <boost/bind.hpp>

namespace gazebo {

    GazeboRosActuator::GazeboRosActuator() {}

    // Destructor
    GazeboRosActuator::~GazeboRosActuator() {
        delete rosnode_;
    }

    // Load the controller
    void GazeboRosActuator::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent = _parent;
        this->world = _parent->GetWorld();

        this->robot_namespace_ = "";
        if (!_sdf->HasElement("robotNamespace")) {
            ROS_INFO("GazeboRosActuator Plugin missing <robotNamespace>, defaults to \"%s\"",
                     this->robot_namespace_.c_str());
        } else {
            this->robot_namespace_ =
                    _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        }

        this->joint_name_ = "actuator_base_link_to_actuator_part1_joint";
        if (!_sdf->HasElement("Joint")) {
            ROS_WARN("GazeboRosActuator Plugin (ns = %s) missing <Joint>, defaults to \"%s\"",
                     this->robot_namespace_.c_str(), this->joint_name_.c_str());
        } else {
            this->joint_name_ = _sdf->GetElement("Joint")->Get<std::string>();
        }

        this->torque = 5.0;
        if (!_sdf->HasElement("torque")) {
            ROS_WARN("GazeboRosActuator Plugin (ns = %s) missing <torque>, defaults to %f",
                     this->robot_namespace_.c_str(), this->torque);
        } else {
            this->torque = _sdf->GetElement("torque")->Get<double>();
        }

        this->command_topic_ = "/oz440/cmd_act";
        if (!_sdf->HasElement("commandTopic")) {
            ROS_WARN("GazeboRosActuator Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                     this->robot_namespace_.c_str(), this->command_topic_.c_str());
        } else {
            this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
        }

        // Initialize velocity stuff
        position_ = 0.0;
        connection_ = 0;

        joints = this->parent->GetJoint(joint_name_);

        if (!joints) {

            char error[200];
            snprintf(error, 200,
                     "GazeboRosActuator Plugin (ns = %s) couldn't get joint named \"%s\"",
                     this->robot_namespace_.c_str(), this->joint_name_.c_str());
            gzthrow(error);
        }

//        joints->SetParam( "fmax", 0, torque);

//        joints->SetPosition( 0, 0.0);

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        rosnode_ = new ros::NodeHandle(this->robot_namespace_);

        ROS_INFO("Starting GazeboRosActuator Plugin (ns = %s)!", this->robot_namespace_.c_str());

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        cmd_subscriber_ = rosnode_->subscribe(this->command_topic_, 50, &GazeboRosActuator::cmdCallback, this);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosActuator::OnUpdate, this));
    }

    void GazeboRosActuator::OnUpdate()
    {
//        if (connection_==1)
//        {
//            joints->SetPosition( 0, position_ );
//            connection_ = 0;
//        }

        joints->SetPosition( 0, position_ );

    }

    void GazeboRosActuator::cmdCallback( const geometry_msgs::Vector3::ConstPtr& cmd_msg)
    {
//        connection_ = 1;
        position_ = cmd_msg->x;

    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosActuator)
}
