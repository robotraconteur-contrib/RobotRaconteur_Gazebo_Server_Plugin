/*
 * Copyright (C) 2016-2020 Wason Technology, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <RobotRaconteur.h>
#include "experimental__gazebo.h"
#include "experimental__gazebo_stubskel.h"

#pragma once

using namespace gazebo;
namespace RR=RobotRaconteur;
namespace rrgz=experimental::gazebo;

#include "RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h"

#include "ServerImpl.h"
#include "WorldImpl.h"
#include "EntityImpl.h"
#include "JointImpl.h"
#include "LinkImpl.h"
#include "ModelImpl.h"
#include "JointControllerImpl.h"
#include "KinematicJointControllerImpl.h"
#include "SensorImpl.h"
#include "CameraImpl.h"
#include "MultiCameraSensorImpl.h"
#include "DepthCameraSensorImpl.h"
#include "RaySensorImpl.h"
#include "GpuRaySensorImpl.h"
#include "ContactSensorImpl.h"
#include "AltimeterSensorImpl.h"
#include "SonarSensorImpl.h"
#include "MagnetometerSensorImpl.h"
#include "ForceTorqueSensorImpl.h"
#include "GpsSensorImpl.h"
#include "ImuSensorImpl.h"
#include "LightImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class RobotRaconteurGazeboServerPlugin : public SystemPlugin
  {
  public:
	RobotRaconteurGazeboServerPlugin();
	~RobotRaconteurGazeboServerPlugin();

	void Load(int /*_argc*/, char ** /*_argv*/);
	void Init();
	void Reset();

  protected:

	boost::mutex this_lock;

	//RR_SHARED_PTR<RR::RobotRaconteurNode> rr_node;
	RR_SHARED_PTR<RR::LocalTransport> rr_local_transport;
	RR_SHARED_PTR<RR::TcpTransport> rr_tcp_transport;

	RR_SHARED_PTR<RR::RobotRaconteurNode> rr_node;

	std::string nodename;
	RR::NodeID nodeid;

	int tcp_port;
	bool tcp_port_sharer;
	bool tcp_node_announce;
	bool tcp_load_tls;

	std::string password_file;

	void rr_start();
	void rr_stop();

	RR_SHARED_PTR<ServerImpl> server;

	static std::string ReadFile(const std::string& fname);

	RR_BOOST_ASIO_IO_CONTEXT io_context;
	RR_SHARED_PTR<RR::IOContextThreadPool> thread_pool;

	void on_world_update_begin(const common::UpdateInfo& world_info);
	event::ConnectionPtr on_world_update_begin_connection;

  };



}
