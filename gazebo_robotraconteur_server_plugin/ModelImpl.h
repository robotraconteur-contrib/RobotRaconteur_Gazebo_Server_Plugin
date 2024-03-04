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

#include "EntityImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class JointControllerImpl;
  class KinematicJointControllerImpl;
  class ModelImpl : public virtual rrgz::Model_default_abstract_impl, public virtual EntityImpl
  {
  public:
	  ModelImpl(physics::ModelPtr m);

	  virtual RR::RRListPtr<RR::RRArray<char> > get_child_model_names() override; 

	  virtual rrgz::ModelPtr get_child_models(const std::string& ind) override;

	  virtual RR::RRListPtr<RR::RRArray<char> > get_link_names() override;
	  
	  virtual rrgz::LinkPtr get_links(const std::string& ind) override;

	  virtual RR::RRListPtr<RR::RRArray<char> > get_joint_names() override;
	  
	  virtual rrgz::JointPtr get_joints(const std::string& ind) override;
	  
	  virtual void create_joint_controller() override;
	  virtual void destroy_joint_controller() override;

	  virtual rrgz::JointControllerPtr get_joint_controller() override;

	  virtual void create_kinematic_joint_controller() override;
	  virtual void destroy_kinematic_joint_controller() override;

	  virtual rrgz::JointControllerPtr get_kinematic_joint_controller() override;

	  virtual std::string RRType() override { return Model_default_abstract_impl::RRType(); }

	  void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

  protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  physics::WorldPtr gz_world;

	  physics::ModelPtr get_model();
	  
	  RR_SHARED_PTR<JointControllerImpl> joint_controller;
	  RR_SHARED_PTR<KinematicJointControllerImpl> kinematic_joint_controller;

  };

}
