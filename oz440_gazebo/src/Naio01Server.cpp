#include "ros/ros.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include "oz440_msgs/Screen.h"
#include "oz440_msgs/Motors.h"
#include "nav_msgs/Odometry.h"
#include "../include/oz440_api/CLBuffer.hpp"

/**
* Simulator input
*/
#include "../include/oz440_api/HaMotorsPacket.hpp"
#include "../include/oz440_api/HaActuatorPacket.hpp"
#include "../include/oz440_api/HaScreenPacket.hpp"
#include "../include/oz440_api/HaLedPacket.hpp"
/**
* Simulator output
*/
#include "../include/oz440_api/HaLidarPacket.hpp"
#include "../include/oz440_api/HaOdoPacket.hpp"
#include "../include/oz440_api/HaGpsPacket.hpp"
#include "../include/oz440_api/HaGyroPacket.hpp"
#include "../include/oz440_api/HaAcceleroPacket.hpp"
#include "../include/oz440_api/HaStereoCameraPacket.hpp"
//#include "../include/oz440_api/HaMagnetoPacket.hpp"


#include "../include/oz440_api/ApiIhmDisplayPacket.hpp"
#include "../include/oz440_api/Naio01Codec.hpp"
#include "../include/oz440_socket/ServerSocket.h"
#include "../include/oz440_socket/SocketException.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/Image.h"

#include "gazebo_msgs/LinkStates.h"

#include <sstream>
#include <mutex>
#include <vector>
#include <sys/types.h>

#include "/home/fanny/catkin_ws/src/oz440_simulator/oz440_gazebo/include/Naio01Server.hpp"

#include <tf/transform_listener.h>
#include <stdio.h>


#define IMAGE_SIZE 270700
#define HEADER_SIZE 15

/**
 * This node aim to listen the naio01 bus from a running server socket
 * and publish data throught topics
 */

using namespace message_filters;

int naio01_server_port = 9999;
ServerSocket * serverSockPtr;
ServerSocket * newSockPtr;
int odom_current_value_fr = 0;
int odom_current_value_fl = 0;
int odom_current_value_rr = 0;
int odom_current_value_rl = 0;



//                    else if( std::dynamic_pointer_cast<HaActuatorPacket>( basePacketPtr ) )
//                    {
//                      /**
//                       *  When receiving an actuator order
//                       */
//                      HaActuatorPacketPtr actuatorPacketPtr = std::dynamic_pointer_cast<HaActuatorPacket>( basePacketPtr );
//                      //TODO envoyer vers gazebo via apply_joint_torque
//                    }
//                    else if (std::dynamic_pointer_cast<ApiIhmDisplayPacket>(basePacketPtr)) {
//                        /**
//                         *  When receiving a display message
//                         */
//                        ApiIhmDisplayPacketPtr displayPacketPtr = std::dynamic_pointer_cast<ApiIhmDisplayPacket>(
//                                basePacketPtr);
//                        ROS_INFO("ApiIhmDisplayPacket received");
//                        oz440_msgs::Screen screen_msg;
//                        for (int j = 0; j < 15; j++) {
//                            screen_msg.topLine[j] = displayPacketPtr->topLine[j];
//                            screen_msg.bottomLine[j] = displayPacketPtr->bottomLine[j];
//                        }
//                        screen_pub.publish(screen_msg);
//                    }
//                }
//                receivedPacketList.clear();
//            }
//            ROS_INFO("Avant Spin");
//            ros::spinOnce();
//            ROS_INFO("Apres Spin");
//        }
//    }
//    catch( SocketException& ) {}
//  }
//  catch (SocketException& e )
//  {
//    ROS_ERROR("Socket exception was caught : %s",e.description().c_str());
//  }
//
//    return 0;
//}
