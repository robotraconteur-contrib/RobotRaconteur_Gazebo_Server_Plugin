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
  class LinkImpl : public virtual EntityImpl, public virtual rrgz::Link
  {
  public:
	  LinkImpl(physics::LinkPtr l);
	 
      virtual std::string get_Name() override {return EntityImpl::get_Name();}
	  
	  virtual std::string get_ScopedName() override {return EntityImpl::get_ScopedName();}
	  
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

	  virtual RR::RRListPtr<RR::RRArray<double > > get_AppliedWrenches() override;
	  virtual void set_AppliedWrenches(RR::RRListPtr<RR::RRArray<double > > value) override;
	  
	  virtual RR::WirePtr<RR::RRListPtr<RR::RRArray<double > > > get_AppliedWrenchesSetWire() override;
	  virtual void set_AppliedWrenchesSetWire(RR::WirePtr<RR::RRListPtr<RR::RRArray<double> > > value) override;

	  virtual RR::RRListPtr<RR::RRArray<char> > get_SensorNames() override;

  protected:
  	  boost::weak_ptr<physics::Link> gz_link;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;

  	  physics::LinkPtr get_link();
  	  virtual physics::EntityPtr get_entity();

  	  RR::RRListPtr<RR::RRArray<double > > applied_wrenches;
  	  virtual void OnUpdate1(const common::UpdateInfo & _info);

  	  RR::WirePtr<RR::RRListPtr<RR::RRArray<double > > > m_AppliedWrenchesSetWire;
  	  RR::WireConnectionPtr<RR::RRListPtr<RR::RRArray<double > > > m_AppliedWrenchesSetWire_conn;
  	  static void OnAppliedWrenchesSetWireConnect(RR_WEAK_PTR<LinkImpl> l, RR::WireConnectionPtr<RR::RRListPtr<RR::RRArray<double > > > connection);
  	  static void OnAppliedWrenchesSetWireDisconnect(RR_WEAK_PTR<LinkImpl> l, RR::WireConnectionPtr<RR::RRListPtr<RR::RRArray<double > > > connection);
  };

}
