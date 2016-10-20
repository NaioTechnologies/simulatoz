/*
 * \file  gazebo_ros_side_drive.h
 *
 * \brief differential drive with four wheels
 *
 * \author  Fanny Risbourg
 *
 */
#include <gazebo_plugins/gazebo_ros_side_drive.h>

#include "gazebo/gazebo.hh"

#include <sdf/sdf.hh>

#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"

#include <boost/bind.hpp>

namespace gazebo {

    enum {
        RIGHT_FRONT=0,
        LEFT_FRONT=1,
        RIGHT_REAR=2,
        LEFT_REAR=3,
    };

    GazeboRosSideDrive::GazeboRosSideDrive() {}

    // Destructor
    GazeboRosSideDrive::~GazeboRosSideDrive() {
        delete rosnode_;
    }

    // Load the controller
    void GazeboRosSideDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent = _parent;
        this->world = _parent->GetWorld();

        this->robot_namespace_ = "";
        if (!_sdf->HasElement("robotNamespace")) {
            ROS_INFO("GazeboRosSkidSteerDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
                     this->robot_namespace_.c_str());
        } else {
            this->robot_namespace_ =
                    _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        }

        this->left_front_joint_name_ = "left_front_joint";
        if (!_sdf->HasElement("leftFrontJoint")) {
            ROS_WARN("GazeboRosSideDrive Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
                     this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
        } else {
            this->left_front_joint_name_ = _sdf->GetElement("leftFrontJoint")->Get<std::string>();
        }

        this->right_front_joint_name_ = "right_front_joint";
        if (!_sdf->HasElement("rightFrontJoint")) {
            ROS_WARN("GazeboRosSideDrive Plugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
                     this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
        } else {
            this->right_front_joint_name_ = _sdf->GetElement("rightFrontJoint")->Get<std::string>();
        }

        this->left_rear_joint_name_ = "left_rear_joint";
        if (!_sdf->HasElement("leftRearJoint")) {
            ROS_WARN("GazeboRosSideDrive Plugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
                     this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
        } else {
            this->left_rear_joint_name_ = _sdf->GetElement("leftRearJoint")->Get<std::string>();
        }

        this->right_rear_joint_name_ = "right_rear_joint";
        if (!_sdf->HasElement("rightRearJoint")) {
            ROS_WARN("GazeboRosSideDrive Plugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
                     this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
        } else {
            this->right_rear_joint_name_ = _sdf->GetElement("rightRearJoint")->Get<std::string>();
        }

        this->wheel_diameter_ = 0.15;
        if (!_sdf->HasElement("wheelDiameter")) {
            ROS_WARN("GazeboRosSideDrive Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
                     this->robot_namespace_.c_str(), this->wheel_diameter_);
        } else {
            this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
        }

        this->torque = 5.0;
        if (!_sdf->HasElement("torque")) {
            ROS_WARN("GazeboRosSideDrive Plugin (ns = %s) missing <torque>, defaults to %f",
                     this->robot_namespace_.c_str(), this->torque);
        } else {
            this->torque = _sdf->GetElement("torque")->Get<double>();
        }

        this->command_topic_ = "/oz440/cmd_vel";
        if (!_sdf->HasElement("commandTopic")) {
            ROS_WARN("GazeboRosSideDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                     this->robot_namespace_.c_str(), this->command_topic_.c_str());
        } else {
            this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
        }

        // Initialize velocity stuff
        wheel_speed_[RIGHT_FRONT] = 0;
        wheel_speed_[LEFT_FRONT] = 0;
        wheel_speed_[RIGHT_REAR] = 0;
        wheel_speed_[LEFT_REAR] = 0;

        x_ = 0;
        y_ = 0;

        joints[LEFT_FRONT] = this->parent->GetJoint(left_front_joint_name_);
        joints[RIGHT_FRONT] = this->parent->GetJoint(right_front_joint_name_);
        joints[LEFT_REAR] = this->parent->GetJoint(left_rear_joint_name_);
        joints[RIGHT_REAR] = this->parent->GetJoint(right_rear_joint_name_);

        if (!joints[LEFT_FRONT]) {

            char error[200];
            snprintf(error, 200,
                     "GazeboRosSideDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
                     this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
            gzthrow(error);
        }

        if (!joints[RIGHT_FRONT]) {
            char error[200];
            snprintf(error, 200,
                     "GazeboRosSideDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
                     this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
            gzthrow(error);
        }

        if (!joints[LEFT_REAR]) {
            char error[200];
            snprintf(error, 200,
                     "GazeboRosSideDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
                     this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
            gzthrow(error);
        }

        if (!joints[RIGHT_REAR]) {
            char error[200];
            snprintf(error, 200,
                     "GazeboRosSideDrivePlugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
                     this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
            gzthrow(error);
        }

        joints[LEFT_FRONT]->SetParam("fmax", 0, torque);
        joints[RIGHT_FRONT]->SetParam("fmax", 0, torque);
        joints[LEFT_REAR]->SetParam("fmax", 0, torque);
        joints[RIGHT_REAR]->SetParam("fmax", 0, torque);

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        rosnode_ = new ros::NodeHandle(this->robot_namespace_);

        ROS_INFO("Starting GazeboRosSideDrive Plugin (ns = %s)!", this->robot_namespace_.c_str());

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        cmd_vel_subscriber_ = rosnode_->subscribe("/oz440/cmd_vel", 50, &GazeboRosSideDrive::cmdVelCallback, this);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosSideDrive::OnUpdate, this));
    }

    void GazeboRosSideDrive::OnUpdate()
    {
        joints[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT] /12 / (wheel_diameter_ / 2.0));
        joints[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT] /12 / (wheel_diameter_ / 2.0));
        joints[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR] / 12 / (wheel_diameter_ / 2.0));
        joints[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR] / 12 / (wheel_diameter_ / 2.0));

    }

    void GazeboRosSideDrive::cmdVelCallback( const geometry_msgs::Vector3::ConstPtr& cmd_msg)
    {
        x_ = cmd_msg->x;
        y_ = cmd_msg->y;

        wheel_speed_[RIGHT_FRONT] = y_;
        wheel_speed_[RIGHT_REAR] =  y_;

        wheel_speed_[LEFT_FRONT] = x_;
        wheel_speed_[LEFT_REAR] = x_;


    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosSideDrive)
}
