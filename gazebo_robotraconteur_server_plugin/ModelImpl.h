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

#include "EntityImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class JointControllerImpl;
  class ModelImpl : public virtual rrgz::Model_default_abstract_impl, public virtual EntityImpl
  {
  public:
	  ModelImpl(physics::ModelPtr m);

	  virtual std::string get_Name() override {return EntityImpl::get_Name();}
	  
	  virtual std::string get_ScopedName() override {return EntityImpl::get_ScopedName();}

	  virtual RR::RRListPtr<RR::RRArray<char> > get_ChildModelNames() override; 

	  virtual rrgz::ModelPtr get_ChildModels(const std::string& ind) override;

	  virtual RR::RRListPtr<RR::RRArray<char> > get_LinkNames() override;
	  
	  virtual rrgz::LinkPtr get_Links(const std::string& ind) override;

	  virtual RR::RRListPtr<RR::RRArray<char> > get_JointNames() override;
	  
	  virtual rrgz::JointPtr get_Joints(const std::string& ind) override;

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

	  virtual void CreateJointController() override;
	  virtual void DestroyJointController() override;

	  virtual rrgz::JointControllerPtr get_JointController() override;

	  virtual std::string RRType() override { return Model_default_abstract_impl::RRType(); }

  protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  physics::WorldPtr gz_world;

	  physics::ModelPtr get_model();
	  virtual physics::EntityPtr get_entity();

	  RR_SHARED_PTR<JointControllerImpl> joint_controller;

  };

}
