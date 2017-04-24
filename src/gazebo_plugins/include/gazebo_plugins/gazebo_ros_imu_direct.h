
/*
 * \file  gazebo_ros_imu_direct.h
 *
 * \brief differential drive with four wheels
 *
 * \author  Fanny Risbourg
 *
 */

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_DIRECT_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_DIRECT_H

// #define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <hector_gazebo_plugins/SetBias.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <dynamic_reconfigure/server.h>

namespace gazebo
{
    class GazeboRosIMUDirect : public ModelPlugin
    {
    public:
        /// \brief Constructor
        GazeboRosIMUDirect();

        /// \brief Destructor
        virtual ~GazeboRosIMUDirect();

    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Reset();
        virtual void Update();

    private:

        int last_gyro_packet_send_;
        /// \brief The parent World
        physics::WorldPtr world;

        /// \brief The link referred to by this plugin
        physics::LinkPtr link;

        /// \brief pointer to ros node
        ros::NodeHandle* node_handle_;
        ros::Publisher pub_;
        ros::Publisher bias_pub_;

        /// \brief ros message
        sensor_msgs::Imu imuMsg;
        sensor_msgs::Imu biasMsg;

        /// \brief store link name
        std::string link_name_;

        /// \brief frame id
        std::string frame_id_;

        /// \brief topic name
        std::string topic_;
        std::string bias_topic_;

        /// \brief allow specifying constant xyz and rpy offsets
        math::Pose offset_;

        /// \brief Sensor models
        SensorModel3 accelModel;
        SensorModel3 rateModel;
        SensorModel yawModel;

        /// \brief A mutex to lock access to fields that are used in message callbacks
        boost::mutex lock;

        /// \brief save current body/physics state
        math::Quaternion orientation;
        math::Vector3 velocity;
        math::Vector3 accel;
        math::Vector3 rate;
        math::Vector3 gravity;

        /// \brief Gaussian noise generator
        double GaussianKernel(double mu,double sigma);

        /// \brief for setting ROS name space
        std::string namespace_;

        /// \brief call back when using service
        bool ServiceCallback(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res);
        ros::ServiceServer srv_;
        std::string serviceName;

        /// \brief Bias service callbacks
        bool SetAccelBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res);
        bool SetRateBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res);
        ros::ServiceServer accelBiasService;
        ros::ServiceServer rateBiasService;

#ifdef USE_CBQ
        ros::CallbackQueue callback_queue_;
      void CallbackQueueThread();
      boost::thread callback_queue_thread_;
#endif

        UpdateTimer updateTimer;
        common::Time last_update_;
        event::ConnectionPtr updateConnection;

        boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_accel_, dynamic_reconfigure_server_rate_, dynamic_reconfigure_server_yaw_;
    };
}

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
