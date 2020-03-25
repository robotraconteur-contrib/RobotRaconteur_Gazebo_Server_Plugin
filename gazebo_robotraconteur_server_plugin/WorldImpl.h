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
namespace datetime = com::robotraconteur::datetime;

namespace RobotRaconteurGazeboServerPlugin
{
  class WorldImpl : public rrgz::World_default_impl, public RR_ENABLE_SHARED_FROM_THIS<WorldImpl>
  {
  public:
	  WorldImpl(physics::WorldPtr w);

	  virtual void Init(const std::string& rr_path);

	  virtual std::string get_name() override;	  

	  static void OnUpdate(RR_WEAK_PTR<WorldImpl> j, const common::UpdateInfo & _info);

	  virtual RR::RRListPtr<RR::RRArray<char> > get_model_names() override;	  

	  virtual RR::RRListPtr<RR::RRArray<char> > get_light_names() override;
	  	  
	  virtual void set_time(RR::WirePtr<rrgz::WorldTimesPtr> value) override;

	  virtual void set_sim_time(RR::WirePtr<datetime::Duration> value) override;

	  virtual rrgz::ModelPtr get_models(const std::string& ind) override;
	  virtual rrgz::LightPtr get_lights(const std::string& ind) override;

	  physics::WorldPtr get_world();

	  std::string GetRRPath();

  protected:
	  boost::weak_ptr<physics::World> gz_world;
	  	  
	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  event::ConnectionPtr updateConnection;

	  std::string rr_path;

  };

}
