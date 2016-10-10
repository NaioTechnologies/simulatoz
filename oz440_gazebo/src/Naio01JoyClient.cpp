#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include "../include/oz440_api/HaMotorsPacket.hpp"
#include "../include/oz440_api/HaActuatorPacket.hpp"
#include "../include/oz440_api/CLBuffer.hpp"

#include "../include/oz440_socket/ClientSocket.h"
#include "../include/oz440_socket/SocketException.h"

#include <sstream>
#include <mutex>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int naio01_server_port=9999;
int8_t leftspeed=0;
int8_t rightspeed=0;
bool isRequest,isPosition;
uint8_t position;
ClientSocket * clientPtr;

void joyCallback( const sensor_msgs::Joy::ConstPtr& msg )
{
  float x=msg->axes[0];
  float y=msg->axes[1];

  // if (x == -0.00)
  //   x = 0.00;
  // else
  //   x = msg->axes[0];

  // if (y == -0.00)
  //   y = 0.00;
  // else
  //   y = msg->axes[1];

  // if (x < 0 )
  //   rightspeed = abs(x) * 127;
  // else
  //   rightspeed = 0;

  // if (y >0)
  // {
  //   /* code */
  // }
  rightspeed= ( y+x )*127;
  leftspeed= ( y-x )*127;
}

void timerCallback(const ros::TimerEvent&)
{
  try
  {
    HaActuatorPacketPtr actuatorPacketPtr = std::make_shared<HaActuatorPacket>( isRequest, isPosition, position );
    cl::BufferUPtr bufferActuator = actuatorPacketPtr->encode();
    ROS_INFO("Packet send, left : %d right : %d", leftspeed,rightspeed);
    clientPtr->sendToSock(bufferActuator->data(),bufferActuator->size());

    HaMotorsPacketPtr motorsPacketPtr = std::make_shared<HaMotorsPacket>(leftspeed,rightspeed);
    cl::BufferUPtr bufferMotors = motorsPacketPtr->encode();
    ROS_INFO("Packet send, left : %d right : %d", leftspeed,rightspeed);
    clientPtr->sendToSock(bufferMotors->data(),bufferMotors->size());
  }
  catch (SocketException& e )
  {
    ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
  }
}

int main(int argc, char **argv)
{
  std::string str_naio01_server_port;

  ros::init(argc, argv, "naio01joyclient");

  ros::NodeHandle n;

  if (!n.hasParam("naio01_server_port"))
  {
    ROS_WARN("No port provided, set as default : 9999");
  }else{
    if(!n.getParam("/naio01_server_port",naio01_server_port))
    {
      ROS_WARN("No port provided, set as default : 9999");
    }
  }

  try
  {
    clientPtr = new ClientSocket("127.0.0.1",naio01_server_port);
  }
  catch (SocketException& e )
  {
    ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
  }

  ros::Subscriber sub = n.subscribe("joy", 1, joyCallback);

  ros::Timer timer = n.createTimer(ros::Duration(0.050), timerCallback);

  ros::spin();

  return 0;
}