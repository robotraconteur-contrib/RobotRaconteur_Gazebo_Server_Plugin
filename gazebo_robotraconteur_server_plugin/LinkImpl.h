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
namespace geometry = com::robotraconteur::geometry;

#include "EntityImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class LinkImpl : public virtual EntityImpl, public virtual rrgz::Link_default_impl
  {
  public:
	  LinkImpl(physics::LinkPtr l);
	 
      virtual std::string get_Name() override {return EntityImpl::get_Name();}
	  
	  virtual std::string get_ScopedName() override {return EntityImpl::get_ScopedName();}

	  virtual void SetWorldPose(const geometry::Pose& value) override { EntityImpl::SetWorldPose(value); }

	  virtual void SetRelativePose(const geometry::Pose& value) override { EntityImpl::SetRelativePose(value); }

	  virtual RR::WirePtr<geometry::Pose> get_WorldPose() override { return EntityImpl::get_WorldPose(); }
	  virtual void set_WorldPose(RR::WirePtr<geometry::Pose> value) override { EntityImpl::set_WorldPose(value); }

	  virtual RR::WirePtr<geometry::Pose> get_RelativePose() override { return EntityImpl::get_RelativePose(); }
	  virtual void set_RelativePose(RR::WirePtr<geometry::Pose> value) override { EntityImpl::set_RelativePose(value); }

	  virtual RR::WirePtr<geometry::SpatialVelocity> get_WorldVelocity() override { return EntityImpl::get_WorldVelocity(); }
	  virtual void set_WorldVelocity(RR::WirePtr<geometry::SpatialVelocity> value) override { EntityImpl::set_WorldVelocity(value); }

	  virtual RR::WirePtr<geometry::SpatialVelocity> get_RelativeVelocity() override { return EntityImpl::get_RelativeVelocity(); }
	  virtual void set_RelativeVelocity(RR::WirePtr<geometry::SpatialVelocity> value) override { EntityImpl::set_RelativeVelocity(value); }

	  virtual RR::WirePtr<geometry::SpatialAcceleration> get_WorldAcceleration() override { return EntityImpl::get_WorldAcceleration(); }
	  virtual void set_WorldAcceleration(RR::WirePtr<geometry::SpatialAcceleration> value) override { EntityImpl::set_WorldAcceleration(value); }

	  virtual RR::WirePtr<geometry::SpatialAcceleration> get_RelativeAcceleration() override { return EntityImpl::get_RelativeAcceleration(); }
	  virtual void set_RelativeAcceleration(RR::WirePtr<geometry::SpatialAcceleration> value) override { EntityImpl::set_RelativeAcceleration(value); }
	 	  
	  virtual RR::RRListPtr<RR::RRArray<char> > get_SensorNames() override;

	  virtual std::string RRType() { return Link_default_impl::RRType(); }

  protected:
  	  boost::weak_ptr<physics::Link> gz_link;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;

  	  physics::LinkPtr get_link();
  	  virtual physics::EntityPtr get_entity();

  	  RR::RRListPtr<RR::RRArray<double > > applied_wrenches;
  	  virtual void OnUpdate1(const common::UpdateInfo & _info);
  };

}
