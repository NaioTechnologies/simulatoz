
//==================================================================================================
//
//  Copyright(c)  2016  Naïo Technologies
//
//  These coded instructions, statements, and computer programs contain unpublished proprietary
//  information written by Naio Technologies and are protected by copyright law. They may not be
//  disclosed to third parties or copied or duplicated in any form, in whole or in part, without
//  the prior written consent of Naio Technologies.
//
//==================================================================================================

#ifndef SIMULATOZ_CAN_H
#define SIMULATOZ_CAN_H

//==================================================================================================
// I N C L U D E   F I L E S

#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include <unistd.h>
#include <net/if.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <mutex>
#include <atomic>
#include <thread>

#include "GeoAngle.hpp"

#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <tf/transform_listener.h>

#include <linux/can.h>

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class Can
{
public:

    enum CanMessageId : unsigned char
    {
        CAN_ID_GEN = 0x00,
        CAN_ID_IMU = 0x03,
        CAN_ID_GPS = 0x04,
        CAN_ID_IHM = 0x07,
        CAN_ID_VER = 0x08,
        CAN_ID_TELECO = 0x0b,
    };

    enum CanMessageType  : unsigned char
    {
        CAN_MOT_CONS = 0x00,

        CAN_IMU_ACC = 0x00,
        CAN_IMU_GYRO = 0x01,

        CAN_TELECO_KEYS = 0x01,
        CAN_TELECO_NUM_VERSION = 0x06,

        CAN_GPS_DATA = 0x00,

        CAN_IHM_LCD = 0x00,
        CAN_IHM_BUT = 0x01,

        CAN_VER_CONS = 0x02,
        CAN_VER_POS = 0x01,
    };

    struct Gps_packet {
        double lat;
        double lon;
        double alt;
        uint8_t satUsed;
        uint8_t quality;
        double groundSpeed;
        bool updated;
    };

//-- Methods ---------------------------------------------------------------------------------------

    Can( uint16_t server_port);
    ~Can();

    void subscribe( ros::NodeHandle& node );
    void cleanup();

private:
    void init();

    void connect();
    void read_thread();
    void send_packet( CanMessageId id, CanMessageType id_msg, uint8_t data[], uint8_t len );
    void disconnect();

    void callback_actuator_position( const sensor_msgs::JointState::ConstPtr& joint_states_msg );
    void callback_imu( const sensor_msgs::Imu::ConstPtr& imu_msg );

    void callback_gps( const sensor_msgs::NavSatFix::ConstPtr& gps_fix_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gps_vel_msg );

    void gps_manager();
    double north_bearing( double lat1, double lon1, double lat2, double lon2 );

    // Odometry part
    void odometry_thread();
    double getPitch( std::string wheel);
    bool odo_wheel( uint8_t & wheel, double& pitch, double& pitch_last_tic, int& forward_backward);


//  *********************************************  -- ATTRIBUTS --  ****************************************************

    std::atomic<bool> stop_;

    //  -- THREADS  --
    std::thread connect_thread_;
    std::thread odometry_thread_;
    std::thread read_thread_;

    //  --  SOCKET  --
    uint16_t server_port_;
    bool socket_connected_;
    int server_socket_desc_;
    int socket_desc_;
    std::mutex socket_access_;

    //  --  ROS SUBSCRIBERS --
    ros::Subscriber actuator_position_sub_;
    ros::Subscriber imu_sub_;
    message_filters::Subscriber< sensor_msgs::NavSatFix > gps_fix_sub_;
    message_filters::Subscriber< geometry_msgs::Vector3Stamped >  gps_vel_sub_;
    message_filters::TimeSynchronizer< sensor_msgs::NavSatFix, geometry_msgs::Vector3Stamped > sync_gps_;

    std::shared_ptr< tf::TransformListener > listener_ptr_;

    ros::Publisher actuator_pub_;

    //  --  ODOMETRY  --
    bool last_odo_ticks_[4];
    std::array < uint8_t, 4> last_odo_packet_;

    //  --  ACTUATOR POSITION AND ORDERS  --
    std::mutex tool_position_access_;
    uint8_t tool_position_;
    std::mutex actuator_order_access_;
    uint8_t actuator_order_;
    bool last_order_down_;

    //  --  GPS  --
    std::mutex gps_packet_access_;
    Gps_packet gps_packet_;
    Gps_packet last_gps_packet_;
};

#endif //SIMULATOZ_CAN_H
