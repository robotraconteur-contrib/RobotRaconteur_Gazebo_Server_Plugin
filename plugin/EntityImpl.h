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
class EntityImpl : public virtual rrgz::Entity, public RR_ENABLE_SHARED_FROM_THIS<EntityImpl>
{
public:
	  virtual std::string get_Name() override;
	  
	  virtual std::string get_ScopedName() override;
	  
	  virtual void Init(const std::string& rr_path);

	  virtual std::string GetRRPath();

	  static void OnUpdate(RR_WEAK_PTR<EntityImpl> j, const common::UpdateInfo & _info);

	  virtual rrgz::PosePtr get_WorldPose() override;
	  virtual void set_WorldPose(rrgz::PosePtr value) override;

	  virtual rrgz::PosePtr get_RelativePose() override;
	  virtual void set_RelativePose(rrgz::PosePtr value) override;

	  virtual RR::RRArrayPtr<double> get_WorldVelocity() override;	  

	  virtual RR::RRArrayPtr<double > get_RelativeVelocity() override;
	  
	  virtual RR::RRArrayPtr<double> get_WorldAcceleration() override;
	  
	  virtual RR::RRArrayPtr<double> get_RelativeAcceleration() override;	  

	  virtual RR::WirePtr<rrgz::PosePtr> get_WorldPoseGetWire() override;
	  virtual void set_WorldPoseGetWire(RR::WirePtr<rrgz::PosePtr> value) override;

	  virtual RR::WirePtr<rrgz::PosePtr> get_RelativePoseGetWire() override;
	  virtual void set_RelativePoseGetWire(RR::WirePtr<rrgz::PosePtr> value) override;

	  virtual RR::WirePtr<RR::RRArrayPtr<double> > get_WorldVelocityGetWire() override;
	  virtual void set_WorldVelocityGetWire(RR::WirePtr<RR::RRArrayPtr<double> > value) override;

	  virtual RR::WirePtr<RR::RRArrayPtr<double> > get_RelativeVelocityGetWire() override;
	  virtual void set_RelativeVelocityGetWire(RR::WirePtr<RR::RRArrayPtr<double> > value) override;

	  virtual RR::WirePtr<RR::RRArrayPtr<double> > get_WorldAccelerationGetWire() override;
	  virtual void set_WorldAccelerationGetWire(RR::WirePtr<RR::RRArrayPtr<double> > value) override;

	  virtual RR::WirePtr<RR::RRArrayPtr<double > > get_RelativeAccelerationGetWire() override;
	  virtual void set_RelativeAccelerationGetWire(RR::WirePtr<RR::RRArrayPtr<double > > value) override;


protected:
	  virtual physics::EntityPtr get_entity()=0;
	  boost::mutex this_lock;

	  RR::WirePtr<rrgz::PosePtr > m_WorldPoseGetWire;
	  RR::WirePtr<rrgz::PosePtr > m_RelativePoseGetWire;

	  RR::WirePtr<RR::RRArrayPtr<double > > m_WorldVelocityGetWire;
	  RR::WirePtr<RR::RRArrayPtr<double > > m_RelativeVelocityGetWire;
	  RR::WirePtr<RR::RRArrayPtr<double > > m_WorldAccelerationGetWire;
	  RR::WirePtr<RR::RRArrayPtr<double > > m_RelativeAccelerationGetWire;

	  RR::WireBroadcasterPtr<rrgz::PosePtr> m_WorldPoseGetWire_b;
	  RR::WireBroadcasterPtr<rrgz::PosePtr> m_RelativePoseGetWire_b;

	  RR::WireBroadcasterPtr<RR::RRArrayPtr<double > > m_WorldVelocityGetWire_b;
	  RR::WireBroadcasterPtr<RR::RRArrayPtr<double > > m_RelativeVelocityGetWire_b;
	  RR::WireBroadcasterPtr<RR::RRArrayPtr<double > > m_WorldAccelerationGetWire_b;
	  RR::WireBroadcasterPtr<RR::RRArrayPtr<double > > m_RelativeAccelerationGetWire_b;

	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  static void OnEntityDeleted(RR_WEAK_PTR<EntityImpl> t, const std::string& path);

	  event::ConnectionPtr updateConnection;
	  event::ConnectionPtr deleteEntityConnection;

	  std::string gz_path;
	  std::string rr_path;
	  RR_WEAK_PTR<RR::ServerContext> rr_context;
};

}
