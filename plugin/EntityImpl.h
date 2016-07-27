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
	  virtual std::string get_Name();
	  virtual void set_Name(std::string value);
	  virtual std::string get_ScopedName();
	  virtual void set_ScopedName(std::string value);

	  virtual void Init(const std::string& rr_path);

	  virtual std::string GetRRPath();

	  static void OnUpdate(RR_WEAK_PTR<EntityImpl> j, const common::UpdateInfo & _info);

	  virtual RR_SHARED_PTR<rrgz::Pose > get_WorldPose();
	  virtual void set_WorldPose(RR_SHARED_PTR<rrgz::Pose > value);

	  virtual RR_SHARED_PTR<rrgz::Pose > get_RelativePose();
	  virtual void set_RelativePose(RR_SHARED_PTR<rrgz::Pose > value);

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_WorldVelocity();
	  virtual void set_WorldVelocity(RR_SHARED_PTR<RR::RRArray<double > > value);

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_RelativeVelocity();
	  virtual void set_RelativeVelocity(RR_SHARED_PTR<RR::RRArray<double > > value);

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_WorldAcceleration();
	  virtual void set_WorldAcceleration(RR_SHARED_PTR<RR::RRArray<double > > value);

	  virtual RR_SHARED_PTR<RR::RRArray<double > > get_RelativeAcceleration();
	  virtual void set_RelativeAcceleration(RR_SHARED_PTR<RR::RRArray<double > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > get_WorldPoseGetWire();
	  virtual void set_WorldPoseGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > get_RelativePoseGetWire();
	  virtual void set_RelativePoseGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_WorldVelocityGetWire();
	  virtual void set_WorldVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_RelativeVelocityGetWire();
	  virtual void set_RelativeVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_WorldAccelerationGetWire();
	  virtual void set_WorldAccelerationGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > get_RelativeAccelerationGetWire();
	  virtual void set_RelativeAccelerationGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value);


protected:
	  virtual physics::EntityPtr get_entity()=0;
	  boost::mutex this_lock;

	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > m_WorldPoseGetWire;
	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > m_RelativePoseGetWire;

	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > m_WorldVelocityGetWire;
	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > m_RelativeVelocityGetWire;
	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > m_WorldAccelerationGetWire;
	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > m_RelativeAccelerationGetWire;

	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::Pose > > > m_WorldPoseGetWire_b;
	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::Pose > > > m_RelativePoseGetWire_b;

	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > m_WorldVelocityGetWire_b;
	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > m_RelativeVelocityGetWire_b;
	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > m_WorldAccelerationGetWire_b;
	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > m_RelativeAccelerationGetWire_b;

	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  static void OnEntityDeleted(RR_WEAK_PTR<EntityImpl> t, const std::string& path);

	  event::ConnectionPtr updateConnection;
	  event::ConnectionPtr deleteEntityConnection;

	  std::string gz_path;
	  std::string rr_path;
	  RR_WEAK_PTR<RR::ServerContext> rr_context;
};

}
