/*
 * Copyright (C) 2016-2020 Wason Technology, LLC
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
class EntityImpl : public virtual rrgz::Entity_default_abstract_impl, public virtual RR::IRRServiceObject, public RR_ENABLE_SHARED_FROM_THIS<EntityImpl>
{
public:
	  
	  EntityImpl(physics::EntityPtr e);

	  virtual std::string get_name() override;
	  
	  virtual std::string get_scoped_name() override;
	  
	  virtual std::string GetRRPath();

	  static void OnUpdate(RR_WEAK_PTR<EntityImpl> j, const common::UpdateInfo & _info);

	  static void OnDelete(RR_WEAK_PTR<EntityImpl> j, const std::string& entity);

	  virtual void setf_world_pose(const geometry::Pose& value) override;
	  	  
	  virtual void setf_relative_pose(const geometry::Pose& value) override;
	  		  
	  virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

protected:
	  virtual physics::EntityPtr get_entity();

	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  event::ConnectionPtr updateConnection;
	  event::ConnectionPtr deleteConnection;
	  
	  std::string gz_path;
	  std::string rr_path;
	  RR_WEAK_PTR<RR::ServerContext> rr_context;

	  boost::weak_ptr<physics::Entity> gz_entity;
};

}
