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
  class ModelImpl : public virtual rrgz::Model, public virtual EntityImpl
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

	  virtual rrgz::PosePtr get_WorldPose() override {return EntityImpl::get_WorldPose();}
	  virtual void set_WorldPose(rrgz::PosePtr value) override {EntityImpl::set_WorldPose(value);}

	  virtual rrgz::PosePtr get_RelativePose() override {return EntityImpl::get_RelativePose();}
	  virtual void set_RelativePose(rrgz::PosePtr value) override {EntityImpl::set_RelativePose(value);}

	  virtual RR::RRArrayPtr<double> get_WorldVelocity() override {return EntityImpl::get_WorldVelocity();}

	  virtual RR::RRArrayPtr<double > get_RelativeVelocity() override {return EntityImpl::get_RelativeVelocity();}
	  
	  virtual RR::RRArrayPtr<double> get_WorldAcceleration() override {return EntityImpl::get_WorldAcceleration();}
	  
	  virtual RR::RRArrayPtr<double> get_RelativeAcceleration() override {return EntityImpl::get_RelativeAcceleration();}

	  virtual RR::WirePtr<rrgz::PosePtr> get_WorldPoseGetWire() override {return EntityImpl::get_WorldPoseGetWire();}
	  virtual void set_WorldPoseGetWire(RR::WirePtr<rrgz::PosePtr> value) override {EntityImpl::set_WorldPoseGetWire(value);}

	  virtual RR::WirePtr<rrgz::PosePtr> get_RelativePoseGetWire() override {return EntityImpl::get_RelativePoseGetWire();}
	  virtual void set_RelativePoseGetWire(RR::WirePtr<rrgz::PosePtr> value) override {EntityImpl::set_RelativePoseGetWire(value);}

	  virtual RR::WirePtr<RR::RRArrayPtr<double> > get_WorldVelocityGetWire() override {return EntityImpl::get_WorldVelocityGetWire();}
	  virtual void set_WorldVelocityGetWire(RR::WirePtr<RR::RRArrayPtr<double> > value) override {EntityImpl::set_WorldVelocityGetWire(value);}

	  virtual RR::WirePtr<RR::RRArrayPtr<double> > get_RelativeVelocityGetWire() override {return EntityImpl::get_RelativeVelocityGetWire();}
	  virtual void set_RelativeVelocityGetWire(RR::WirePtr<RR::RRArrayPtr<double> > value) override {EntityImpl::set_RelativeVelocityGetWire(value);}

	  virtual RR::WirePtr<RR::RRArrayPtr<double> > get_WorldAccelerationGetWire() override {return EntityImpl::get_WorldAccelerationGetWire();}
	  virtual void set_WorldAccelerationGetWire(RR::WirePtr<RR::RRArrayPtr<double> > value) override {EntityImpl::set_WorldAccelerationGetWire(value);}

	  virtual RR::WirePtr<RR::RRArrayPtr<double > > get_RelativeAccelerationGetWire() override {return EntityImpl::get_RelativeAccelerationGetWire();}
	  virtual void set_RelativeAccelerationGetWire(RR::WirePtr<RR::RRArrayPtr<double > > value) override {EntityImpl::set_RelativeAccelerationGetWire(value);}

	  virtual void CreateJointController() override;
	  virtual void DestroyJointController() override;

	  virtual rrgz::JointControllerPtr get_JointController() override;

  protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  physics::WorldPtr gz_world;

	  physics::ModelPtr get_model();
	  virtual physics::EntityPtr get_entity();

	  RR_SHARED_PTR<JointControllerImpl> joint_controller;

  };

}
