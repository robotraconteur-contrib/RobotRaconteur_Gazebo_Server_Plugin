cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
project(gazebo_robotraconteur_server_plugin)

set (CMAKE_CXX_STANDARD 11)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4250 /wd4251 /wd4275 /wd4068 /wd4005")
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(Boost COMPONENTS date_time filesystem system regex chrono atomic thread program_options REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteurCompanion REQUIRED)

include_directories(${RobotRaconteur_INCLUDE_DIR})

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS
    org.gazebosim.gazebo.robdef    
	MASTER_HEADER
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
	AUTO_IMPORT
	)

if (MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

add_library(gazebo_robotraconteur_server_plugin SHARED 
	robotraconteur_gazebo_server_plugin.cpp
	robotraconteur_gazebo_server_plugin.h
	${CMAKE_CURRENT_BINARY_DIR}/org__gazebosim__gazebo_stubskel.cpp
	ServerImpl.cpp
	WorldImpl.cpp
	JointImpl.cpp
	JointControllerImpl.cpp
	KinematicJointControllerImpl.cpp
	LinkImpl.cpp
	ModelImpl.cpp
	SensorImpl.cpp
	CameraImpl.cpp
	LogicalCameraImpl.cpp
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

target_link_libraries(gazebo_robotraconteur_server_plugin ${GAZEBO_LIBRARIES} RobotRaconteurCompanion RobotRaconteurCore )

include(GNUInstallDirs)

install(TARGETS gazebo_robotraconteur_server_plugin 
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
