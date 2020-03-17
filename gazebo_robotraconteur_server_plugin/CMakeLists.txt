cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
project(gazebo_robotraconteur_server_plugin)

set (CMAKE_CXX_STANDARD 11)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4250 /wd4251 /wd4275 /wd4068")
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(Boost COMPONENTS date_time filesystem system regex chrono atomic thread program_options REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteur_standard_robdef_cpp REQUIRED)

include_directories(${RobotRaconteur_INCLUDE_DIR})

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS
    experimental.gazebo.robdef	
    com.robotraconteur.geometry.robdef
	com.robotraconteur.image.robdef
	com.robotraconteur.color.robdef
	com.robotraconteur.datetime.robdef	
	com.robotraconteur.resource.robdef
	com.robotraconteur.identifier.robdef
	com.robotraconteur.uuid.robdef
	com.robotraconteur.laserscan.robdef
	com.robotraconteur.pid.robdef
	com.robotraconteur.sensordata.robdef
	com.robotraconteur.gps.robdef
	com.robotraconteur.imu.robdef
	com.robotraconteur.sensor.robdef
	com.robotraconteur.param.robdef
	com.robotraconteur.datatype.robdef
	com.robotraconteur.units.robdef
	com.robotraconteur.device.robdef
	MASTER_HEADER
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
	)

if (MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

add_library(gazebo_robotraconteur_server_plugin SHARED 
	robotraconteur_gazebo_server_plugin.cpp
	robotraconteur_gazebo_server_plugin.h
	${CMAKE_CURRENT_BINARY_DIR}/experimental__gazebo_stubskel.cpp
	ServerImpl.cpp
	WorldImpl.cpp
	JointImpl.cpp
	JointControllerImpl.cpp
	LinkImpl.cpp
	ModelImpl.cpp
	SensorImpl.cpp
	CameraImpl.cpp
	EntityImpl.cpp
	MultiCameraSensorImpl.cpp
	DepthCameraSensorImpl.cpp
	RaySensorImpl.cpp
	GpuRaySensorImpl.cpp
	ContactSensorImpl.cpp
	AltimeterSensorImpl.cpp
	SonarSensorImpl.cpp
	MagnetometerSensorImpl.cpp
	ForceTorqueSensorImpl.cpp
	GpsSensorImpl.cpp
	ImuSensorImpl.cpp
	LightImpl.cpp
	${RR_THUNK_HDRS}
	${RR_THUNK_SRCS}
	)

target_link_libraries(gazebo_robotraconteur_server_plugin ${GAZEBO_LIBRARIES} RobotRaconteurCore ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES})

include(GNUInstallDirs)

install(TARGETS gazebo_robotraconteur_server_plugin DESTINATION ${CMAKE_INSTALL_LIBDIR})
