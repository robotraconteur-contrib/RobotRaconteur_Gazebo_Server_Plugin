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
  class LinkImpl : public virtual rrgz::Link, public virtual EntityImpl
  {
  public:
	  LinkImpl(physics::LinkPtr l);

	  virtual std::string get_Name() {return EntityImpl::get_Name();}
	  virtual void set_Name(std::string n) {return EntityImpl::set_Name(n);}
	  virtual std::string get_ScopedName() {return EntityImpl::get_ScopedName();}
	  virtual void set_ScopedName(std::string n) {return EntityImpl::set_ScopedName(n);}


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

	  virtual RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > get_AppliedWrenches();
	  virtual void set_AppliedWrenches(RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > value);

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

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > > > get_AppliedWrenchesSetWire();
	  virtual void set_AppliedWrenchesSetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > get_SensorNames();
	  virtual void set_SensorNames(RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > value);


  protected:
  	  boost::weak_ptr<physics::Link> gz_link;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;

  	  physics::LinkPtr get_link();
  	  virtual physics::EntityPtr get_entity();

  	  RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > applied_wrenches;
  	  virtual void OnUpdate1(const common::UpdateInfo & _info);

  	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > > > m_AppliedWrenchesSetWire;
  	  RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > > > m_AppliedWrenchesSetWire_conn;
  	  static void OnAppliedWrenchesSetWireConnect(RR_WEAK_PTR<LinkImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > > > connection);
  	  static void OnAppliedWrenchesSetWireDisconnect(RR_WEAK_PTR<LinkImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRList<RR::RRArray<double >  > > > > connection);
  };

}
