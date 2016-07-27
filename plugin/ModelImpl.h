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

	  virtual std::string get_Name() {return EntityImpl::get_Name();}
	  virtual void set_Name(std::string n) {return EntityImpl::set_Name(n);}
	  virtual std::string get_ScopedName() {return EntityImpl::get_ScopedName();}
	  virtual void set_ScopedName(std::string n) {return EntityImpl::set_ScopedName(n);}

	  virtual RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > get_ChildModelNames();
	  virtual void set_ChildModelNames(RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > value);

	  virtual RR_SHARED_PTR<rrgz::Model> get_ChildModels(std::string ind);

	  virtual RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > get_LinkNames();
	  virtual void set_LinkNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value);

	  virtual RR_SHARED_PTR<rrgz::Link > get_Links(std::string ind);

	  virtual RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > get_JointNames();
	  virtual void set_JointNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value);

	  virtual RR_SHARED_PTR<rrgz::Joint > get_Joints(std::string ind);

	  virtual RR_SHARED_PTR<rrgz::Pose > get_WorldPose() {return EntityImpl::get_WorldPose(); }
	  virtual void set_WorldPose(RR_SHARED_PTR<rrgz::Pose > value) { EntityImpl::set_WorldPose(value); }

	  virtual RR_SHARED_PTR<rrgz::Pose > get_RelativePose() {return EntityImpl::get_RelativePose();}
	  virtual void set_RelativePose(RR_SHARED_PTR<rrgz::Pose > value) {EntityImpl::set_RelativePose(value);}

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_WorldVelocity() {return EntityImpl::get_WorldVelocity();}
	  virtual void set_WorldVelocity(RR_SHARED_PTR<RR::RRArray<double > > value) {EntityImpl::set_WorldVelocity(value);}

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_RelativeVelocity() {return EntityImpl::get_RelativeVelocity();}
	  virtual void set_RelativeVelocity(RR_SHARED_PTR<RR::RRArray<double > > value) {EntityImpl::set_RelativeVelocity(value);}

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_WorldAcceleration() {return EntityImpl::get_WorldAcceleration();}
	  virtual void set_WorldAcceleration(RR_SHARED_PTR<RR::RRArray<double > > value) {EntityImpl::set_WorldAcceleration(value);}

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_RelativeAcceleration() {return EntityImpl::get_RelativeAcceleration();}
	  virtual void set_RelativeAcceleration(RR_SHARED_PTR<RR::RRArray<double > > value) {EntityImpl::set_RelativeAcceleration(value);}

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > get_WorldPoseGetWire() {return EntityImpl::get_WorldPoseGetWire();}
	  virtual void set_WorldPoseGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > value) {EntityImpl::set_WorldPoseGetWire(value);}

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > get_RelativePoseGetWire() {return EntityImpl::get_RelativePoseGetWire();}
	  virtual void set_RelativePoseGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > value) {EntityImpl::set_RelativePoseGetWire(value);}

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_WorldVelocityGetWire() {return EntityImpl::get_WorldVelocityGetWire();}
	  virtual void set_WorldVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value) {EntityImpl::set_WorldVelocityGetWire(value);}

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_RelativeVelocityGetWire() {return EntityImpl::get_RelativeVelocityGetWire();}
	  virtual void set_RelativeVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value) {EntityImpl::set_RelativeVelocityGetWire(value);}

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_WorldAccelerationGetWire() {return EntityImpl::get_WorldAccelerationGetWire();}
	  virtual void set_WorldAccelerationGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value) {EntityImpl::set_WorldAccelerationGetWire(value);}

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_RelativeAccelerationGetWire() {return EntityImpl::get_RelativeAccelerationGetWire();}
	  virtual void set_RelativeAccelerationGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value) {EntityImpl::set_RelativeAccelerationGetWire(value);}

	  virtual void CreateJointController();
	  virtual void DestroyJointController();

	  virtual RR_SHARED_PTR<rrgz::JointController > get_JointController();

  protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  physics::WorldPtr gz_world;

	  physics::ModelPtr get_model();
	  virtual physics::EntityPtr get_entity();

	  RR_SHARED_PTR<JointControllerImpl> joint_controller;

  };

}
