cmake_minimum_required( VERSION 3.0.0 )

project( oz440_gazebo )

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
include( CheckCXXCompilerFlag )

check_cxx_compiler_flag( "-std=c++1y" COMPILER_SUPPORTS_CXX11 )

if( COMPILER_SUPPORTS_CXX11 )
	set( CMAKE_CXX_FLAGS "-std=c++1y" )
elseif( COMPILER_SUPPORTS_CXX0X )
	set( CMAKE_CXX_FLAGS "-std=c++0x" )
else()
	message( FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler." )
endif()

find_package( catkin )
catkin_package()

find_package( OpenCV 2.4.5 )

find_package( gazebo REQUIRED )
find_package( catkin REQUIRED COMPONENTS
			  roscpp
			  tf
			  gazebo_ros
			  std_msgs
			  sensor_msgs
			  oz440_msgs
			  oz440_api
			  gazebo_msgs
			  image_transport
              cv_bridge  )

include_directories( include
					 ${catkin_INCLUDE_DIRS}
					 ${oz440_api_INCLUDE_DIRS} )

add_executable( Core src/Core.cpp
				src/GeoAngle.cpp
				src/DriverSocket.cpp
				src/Camera.cpp
				src/Lidar.cpp
				src/Can.cpp
				src/Serial.cpp
				src/Metric.cpp
				src/Log.cpp
				src/VideoLog.cpp
				src/Odometry.cpp
				src/Gps.cpp)

target_link_libraries( Core ${catkin_LIBRARIES}
					   oz440_api
					   -lpthread
					   -lz
					   ${OpenCV_LIBRARIES} )

add_dependencies( Core oz440_api ${catkin_EXPORTED_TARGETS} )
