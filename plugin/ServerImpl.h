/*
 * Copyright (C) 2016 Wason Technology, LLC
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

namespace RobotRaconteurGazeboServerPlugin
{

  class WorldImpl;

  class ServerImpl : public rrgz::Server, public RR_ENABLE_SHARED_FROM_THIS<ServerImpl>
  {
  public:

	  virtual void Init();

	  virtual RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > get_WorldNames();
	  virtual void set_WorldNames(RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > value);

	  virtual RR_SHARED_PTR<rrgz::World > get_Worlds(std::string ind);


	  virtual RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > get_SensorNames();
	  virtual void set_SensorNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value);


	  virtual RR_SHARED_PTR<rrgz::Sensor > get_Sensors(std::string ind);

  protected:

	  void OnWorldCreated(std::string name);

	  std::map<std::string,RR_SHARED_PTR<WorldImpl> > rr_worlds;

	  boost::mutex this_lock;
	  event::ConnectionPtr worldCreatedConnection;
  };
}
