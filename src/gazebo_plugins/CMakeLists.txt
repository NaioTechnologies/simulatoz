cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  hector_gazebo_plugins
)

find_package(gazebo REQUIRED
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} include ${GAZEBO_INCLUDE_DIRS} )

add_library( gazebo_ros_side_drive src/gazebo_ros_side_drive.cpp

			 )
#add_library(gazebo_ros_imu_direct src/gazebo_ros_imu_direct.cpp
#
#)
add_library( gazebo_ros_actuator src/gazebo_ros_actuator.cpp

			 )

target_link_libraries( gazebo_ros_actuator  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
        )
#target_link_libraries( gazebo_ros_imu_direct  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
#        )
target_link_libraries( gazebo_ros_side_drive  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
        )

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
