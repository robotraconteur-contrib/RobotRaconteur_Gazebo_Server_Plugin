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
namespace pid = com::robotraconteur::pid;

namespace RobotRaconteurGazeboServerPlugin
{
class ModelImpl;
class JointControllerImpl : public virtual rrgz::JointController_default_impl, public virtual RR_ENABLE_SHARED_FROM_THIS<JointControllerImpl>
{
public:

	  JointControllerImpl(RR_SHARED_PTR<ModelImpl> model, physics::ModelPtr gz_model);

	  virtual void Init();

	  virtual RR::RRListPtr<RR::RRArray<char> > get_joint_names() override;	  

	  virtual RR::RRMapPtr<std::string,pid::PIDParam> get_position_pid() override;

	  virtual RR::RRMapPtr<std::string,pid::PIDParam> get_velocity_pid() override;	  

	  virtual void add_joint(const std::string& name) override;

	  virtual void setf_position_pid(const std::string& name, pid::PIDParamPtr pid) override;

	  virtual void setf_velocity_pid(const std::string& name, pid::PIDParamPtr pid) override;

	  virtual void set_joint_position(RR::WirePtr<RR::RRMapPtr<std::string, RR::RRArray<double > > > value) override;
	  
	  virtual void set_joint_velocity(RR::WirePtr<RR::RRMapPtr<std::string, RR::RRArray<double > > > value) override;
	  
	  virtual void set_joint_forces(RR::WirePtr<RR::RRMapPtr<std::string, RR::RRArray<double > > > value) override;

protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  boost::shared_ptr<physics::JointController> gz_controller;
	  	  	  
	  virtual physics::ModelPtr get_model();
	  	  
	  static void OnUpdate(RR_WEAK_PTR<JointControllerImpl> j, const common::UpdateInfo& _info);
	  void OnUpdate1(const common::UpdateInfo& _info);

	  event::ConnectionPtr updateConnection;

};

}
