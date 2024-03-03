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
#include "org__gazebosim__gazebo.h"
#include "org__gazebosim__gazebo_stubskel.h"

#pragma once

using namespace gazebo;
namespace RR=RobotRaconteur;
namespace rrgz=org::gazebosim::gazebo;
namespace pid = com::robotraconteur::pid;

namespace RobotRaconteurGazeboServerPlugin
{
class ModelImpl;
class KinematicJointControllerImpl : public virtual rrgz::JointController_default_impl, public virtual RR::IRRServiceObject, public virtual RR_ENABLE_SHARED_FROM_THIS<KinematicJointControllerImpl>
{
public:

	  KinematicJointControllerImpl(RR_SHARED_PTR<ModelImpl> model, physics::ModelPtr gz_model);

	  virtual RR::RRListPtr<RR::RRArray<char> > get_joint_names() override;	  

	  virtual RR::RRMapPtr<std::string,pid::PIDParam> get_position_pid() override;

	  virtual RR::RRMapPtr<std::string,pid::PIDParam> get_velocity_pid() override;	  

	  virtual void add_joint(const std::string& name) override;

	  virtual void setf_position_pid(const std::string& name, const pid::PIDParamPtr& pid) override;

	  virtual void setf_velocity_pid(const std::string& name, const pid::PIDParamPtr& pid) override;

	  virtual com::robotraconteur::device::isoch::IsochInfoPtr get_isoch_info() override;

	  virtual uint32_t get_isoch_downsample() override;

	  virtual void set_isoch_downsample(uint32_t value) override;

	  virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

	  virtual std::string RRPath();

	  virtual void _set_joint_target(const std::string& name, const RR::RRArrayPtr<double>& value);

protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  std::map<std::string,boost::weak_ptr<physics::Joint> > joints;
	  RR::RRMapPtr<std::string,RR::RRArray<double> > joint_targets;
	  RR::RRMapPtr<std::string,RR::RRArray<double> > joint_velocities;
	  	  	  
	  virtual physics::ModelPtr get_model();
	  	  
	  static void OnUpdate(RR_WEAK_PTR<KinematicJointControllerImpl> j, const common::UpdateInfo& _info);
	  void OnUpdate1(const common::UpdateInfo& _info);

	  event::ConnectionPtr updateConnection;

	  std::string rr_path;

	  RR::BroadcastDownsamplerPtr rr_downsampler;

};

}
