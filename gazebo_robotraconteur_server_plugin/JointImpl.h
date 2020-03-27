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
namespace geometry=com::robotraconteur::geometry;

namespace RobotRaconteurGazeboServerPlugin
{
  class JointImpl : public virtual rrgz::Joint_default_impl, public virtual RR::IRRServiceObject, public RR_ENABLE_SHARED_FROM_THIS<JointImpl>
  {
  public:
	  JointImpl(physics::JointPtr j);

	  static void OnUpdate(RR_WEAK_PTR<JointImpl> j, const common::UpdateInfo & _info);


	  virtual std::string get_name() override; 
	  virtual std::string get_scoped_name() override;  

	  virtual std::string get_parent_link_name() override;	  

	  virtual std::string get_child_link_name() override;	  

	  virtual uint32_t get_dof() override;
	  
	  virtual RR::RRNamedArrayPtr<geometry::Vector3 > getf_global_axes() override;

	  virtual RR::RRNamedArrayPtr<geometry::Vector3> getf_local_axes() override;

	  virtual void setf_axis_position(uint32_t axis, double Position);

	  virtual void setf_axis_velocity(uint32_t axis, double vel);

	  virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

	 
  protected:

  	  std::string link_name;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;
  	  boost::weak_ptr<physics::Joint> gz_joint;

  	 physics::JointPtr get_joint();

  	 virtual void OnUpdate1(const common::UpdateInfo & _info);

  	 event::ConnectionPtr updateConnection;

  	 RR::RRArrayPtr<double> axes_forces;

	 std::string rr_path;
	 RR_WEAK_PTR<RR::ServerContext> rr_context;	
  };

}
