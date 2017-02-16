cmake_minimum_required(VERSION 2.8.3)
project(oz440_api)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++1y")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories( include ${CATKIN_INCLUDE_DIRS} )

set(MY_LIB_SOURCES
	src/ApiAutoStatusPacket.cpp
	src/ApiPressedIhmButtonPacket.cpp
	src/HaGpsPacket.cpp
	src/ApiCommandPacket.cpp
	src/ApiRunPlotPacket.cpp
	src/HaGyroPacket.cpp
	src/ApiEnumResponsePacket.cpp
	src/ApiSmsPacket.cpp
	src/HaKeypadPacket.cpp
	src/ApiGprsPacket.cpp
	src/ApiStatusPacket.cpp
	src/HaLedPacket.cpp
	src/ApiGpsPacket.cpp
	src/ApiStereoCameraPacket.cpp
	src/HaLidarPacket.cpp
	src/ApiIhmAskEnumPacket.cpp
	src/ApiValueResponsePacket.cpp
	src/HaMagnetoPacket.cpp
	src/ApiIhmAskValuePacket.cpp
	src/ApiWatchdogPacket.cpp
	src/HaMotorsPacket.cpp
	src/ApiIhmDisplayPacket.cpp
	src/BaseNaio01Packet.cpp
	src/HaOdoPacket.cpp
	src/ApiLidarPacket.cpp
	src/CLBuffer.cpp
	src/HaScreenPacket.cpp
	src/ApiLogToRobotPacket.cpp
	src/CLByteConversion.cpp
	src/HaSpeakerPacket.cpp
	src/ApiMessagePacket.cpp
	src/HaAcceleroPacket.cpp
	src/Naio01Codec.cpp
	src/ApiMotorsPacket.cpp
	src/HaActuatorPacket.cpp
	src/ApiMoveActuatorPacket.cpp
	src/HaCanPacket.cpp
	src/ApiPostPacket.cpp
	src/HaDS4RemotePacket.cpp
	src/Utils.cpp
	src/CLException.cpp
	src/CLBaseException.cpp
	src/CLFileSystem.cpp
        )

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} ${MY_LIB_SOURCES})

catkin_package( INCLUDE_DIRS include
				LIBRARIES ${PROJECT_NAME} )

install( DIRECTORY include
		 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

