cmake_minimum_required(VERSION 2.8.3)
project(oz440_gazebo)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++1y")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(MY_LIB_DIR ${CMAKE_SOURCE_DIR}/../devel/lib/)
set(OZ_API ${MY_LIB_DIR}liboz440_api.so)
set(OZ_SOCKET ${MY_LIB_DIR}liboz440_socket.so)

catkin_package()

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  gazebo_ros
  std_msgs
  sensor_msgs
  oz440_msgs
  oz440_api
  oz440_socket
  gazebo_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_library(${OZ_API} liboz440_api)
find_library(${OZ_SOCKET} liboz440_socket)

add_executable(Core  src/Core.cpp)
target_link_libraries(Core ${catkin_LIBRARIES} ${OZ_API} ${OZ_SOCKET})
add_dependencies(Core oz440_api oz440_socket ${catkin_EXPORTED_TARGETS})

add_executable(Naio01JoyClient  src/Naio01JoyClient.cpp)
target_link_libraries(Naio01JoyClient ${catkin_LIBRARIES} ${OZ_API} ${OZ_SOCKET})
add_dependencies(Naio01JoyClient oz440_api oz440_socket ${catkin_EXPORTED_TARGETS})