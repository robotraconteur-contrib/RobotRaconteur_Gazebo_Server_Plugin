cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(RobotRaconteurGazeboPlugin)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(Boost COMPONENTS date_time filesystem system regex chrono atomic thread program_options REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(RobotRaconteur REQUIRED)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/experimental__gazebo_stubskel.cpp COMMAND ${RobotRaconteur_GEN} ARGS --thunksource --lang=cpp ${CMAKE_CURRENT_SOURCE_DIR}/experimental.gazebo.robdef WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/experimental.gazebo.robdef)

add_library(robotraconteur_gazebo_server_plugin SHARED 
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
	)
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/experimental__gazebo_stubskel.cpp PROPERTIES GENERATED 1)
target_link_libraries(robotraconteur_gazebo_server_plugin ${GAZEBO_LIBRARIES} ${RobotRaconteur_LIBRARY} ${Boost_LIBRARIES} ${RobotRaconteur_EXTRA_LIBRARIES})
