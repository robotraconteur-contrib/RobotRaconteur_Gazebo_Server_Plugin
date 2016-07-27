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
  class WorldImpl : public rrgz::World, public RR_ENABLE_SHARED_FROM_THIS<WorldImpl>
  {
  public:
	  WorldImpl(physics::WorldPtr w);

	  virtual void Init(const std::string& rr_path);

	  virtual std::string get_Name();
	  virtual void set_Name(std::string value);

	  static void OnUpdate(RR_WEAK_PTR<WorldImpl> j, const common::UpdateInfo & _info);

	  virtual double get_SimTime();
	  virtual void set_SimTime(double value);

	  virtual double get_RealTime();
	  virtual void set_RealTime(double value);

	  virtual double get_WallTime();
	  virtual void set_WallTime(double value);

	  virtual double get_StartTime();
	  virtual void set_StartTime(double value);

	  virtual RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > get_ModelNames();
	  virtual void set_ModelNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value);

	  virtual RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > get_LightNames();
	  virtual void set_LightNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value);

	  virtual RR_SHARED_PTR<RR::Wire<double > > get_SimTimeWire();
	  virtual void set_SimTimeWire(RR_SHARED_PTR<RR::Wire<double > > value);

	  virtual RR_SHARED_PTR<RR::Wire<double > > get_RealTimeWire();
	  virtual void set_RealTimeWire(RR_SHARED_PTR<RR::Wire<double > > value);

	  virtual RR_SHARED_PTR<rrgz::Model> get_Models(std::string ind);
	  virtual RR_SHARED_PTR<rrgz::Light> get_Lights(std::string ind);

	  physics::WorldPtr get_world();

	  std::string GetRRPath();

  protected:
	  boost::weak_ptr<physics::World> gz_world;

	  RR_SHARED_PTR<RR::Wire<double > > m_SimTimeWire;
	  RR_SHARED_PTR<RR::Wire<double > > m_RealTimeWire;

	  RR_SHARED_PTR<RR::WireBroadcaster<double > > m_SimTimeWire_b;
	  RR_SHARED_PTR<RR::WireBroadcaster<double > > m_RealTimeWire_b;

	  boost::mutex this_lock;

	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  event::ConnectionPtr updateConnection;

	  std::string rr_path;

  };

}
