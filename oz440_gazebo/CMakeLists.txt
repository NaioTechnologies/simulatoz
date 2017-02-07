cmake_minimum_required(VERSION 2.8.3)
project(oz440_gazebo)

#---------------------------------------------------------------------------------------------------
#
# Set default cmake build type to Debug (None Debug Release RelWithDebInfo)
#
set( CMAKE_BUILD_TYPE "Release" )

if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE "Debug" )
endif()

#---------------------------------------------------------------------------------------------------
#
#   Add builds definitions
#
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    add_definitions( -DDEBUG )
    add_compile_options( -g -march=atom -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -ffast-math
            -ftree-loop-if-convert -funroll-loops -mfpmath=sse )

    if( BUILD_WITH_SANITIZERS )
        add_compile_options( -fno-optimize-sibling-calls -fno-omit-frame-pointer
                -fsanitize=leak -fsanitize=address -fsanitize=undefined
                #[[-fsanitize=thread]] )
    endif( BUILD_WITH_SANITIZERS )

elseif( CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
    add_definitions( -DDEBUG )
    add_compile_options( -O3 -march=atom -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -ffast-math
            -ftree-loop-if-convert -funroll-loops -mfpmath=sse )
elseif( CMAKE_BUILD_TYPE STREQUAL "Release" )
    add_compile_options( -O3 -march=atom -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -ffast-math
            -ftree-loop-if-convert -funroll-loops -mfpmath=sse )
endif()


# check c++11 / c++0x
include(CheckCXXCompilerFlag)


CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX11)

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

add_executable(Core src/Core.cpp
                    src/GeoAngle.cpp
                    src/createLidarTrame.cpp
                    src/DriverSerial.cpp
                    src/Camera.cpp
                    src/Lidar.cpp
                    src/Can.cpp
                    src/Serial.cpp)

target_link_libraries(Core ${catkin_LIBRARIES} ${OZ_API} ${OZ_SOCKET}
        -lpthread
        -lSDL2main
        -lSDL2
        -lSDL2_ttf
        -lSDL2_gfx
        -lz)
add_dependencies(Core oz440_api oz440_socket ${catkin_EXPORTED_TARGETS})
