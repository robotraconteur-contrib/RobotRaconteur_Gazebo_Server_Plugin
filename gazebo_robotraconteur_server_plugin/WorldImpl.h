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

	  virtual std::string get_Name() override;	  

	  static void OnUpdate(RR_WEAK_PTR<WorldImpl> j, const common::UpdateInfo & _info);

	  virtual double get_SimTime() override;
	  
	  virtual double get_RealTime() override;
	  
	  virtual double get_WallTime() override;	  

	  virtual double get_StartTime() override;
	  
	  virtual RR::RRListPtr<RR::RRArray<char> > get_ModelNames() override;	  

	  virtual RR::RRListPtr<RR::RRArray<char> > get_LightNames() override;

	  virtual RR::WirePtr<double > get_SimTimeWire() override;
	  virtual void set_SimTimeWire(RR::WirePtr<double> value) override;

	  virtual RR::WirePtr<double> get_RealTimeWire() override;
	  virtual void set_RealTimeWire(RR::WirePtr<double> value);

	  virtual rrgz::ModelPtr get_Models(const std::string& ind) override;
	  virtual rrgz::LightPtr get_Lights(const std::string& ind) override;

	  physics::WorldPtr get_world();

	  std::string GetRRPath();

  protected:
	  boost::weak_ptr<physics::World> gz_world;

	  RR::WirePtr<double> m_SimTimeWire;
	  RR::WirePtr<double> m_RealTimeWire;

	  RR::WireBroadcasterPtr<double > m_SimTimeWire_b;
	  RR::WireBroadcasterPtr<double > m_RealTimeWire_b;

	  boost::mutex this_lock;

	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  event::ConnectionPtr updateConnection;

	  std::string rr_path;

  };

}
