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

namespace RobotRaconteurGazeboServerPlugin
{
class EntityImpl : public virtual rrgz::Entity_default_impl, public RR_ENABLE_SHARED_FROM_THIS<EntityImpl>
{
public:
	  virtual std::string get_Name() override;
	  
	  virtual std::string get_ScopedName() override;
	  
	  virtual void Init(const std::string& rr_path);

	  virtual std::string GetRRPath();

	  static void OnUpdate(RR_WEAK_PTR<EntityImpl> j, const common::UpdateInfo & _info);

	  virtual void SetWorldPose(const geometry::Pose& value) override;
	  	  
	  virtual void SetRelativePose(const geometry::Pose& value) override;
	  		  
	  virtual void set_WorldPose(RR::WirePtr<geometry::Pose> value) override;
	  	  
	  virtual void set_RelativePose(RR::WirePtr<geometry::Pose> value) override;
	  	  
	  virtual void set_WorldVelocity(RR::WirePtr<geometry::SpatialVelocity> value) override;
	  	  
	  virtual void set_RelativeVelocity(RR::WirePtr<geometry::SpatialVelocity> value) override;
	  	  
	  virtual void set_WorldAcceleration(RR::WirePtr<geometry::SpatialAcceleration> value) override;
	  
	  virtual void set_RelativeAcceleration(RR::WirePtr<geometry::SpatialAcceleration> value) override;

protected:
	  virtual physics::EntityPtr get_entity()=0;
	  boost::mutex this_lock;

	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  static void OnEntityDeleted(RR_WEAK_PTR<EntityImpl> t, const std::string& path);

	  event::ConnectionPtr updateConnection;
	  event::ConnectionPtr deleteEntityConnection;

	  std::string gz_path;
	  std::string rr_path;
	  RR_WEAK_PTR<RR::ServerContext> rr_context;
};

}
