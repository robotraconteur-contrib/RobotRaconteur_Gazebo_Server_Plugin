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
namespace geometry = com::robotraconteur::geometry;

#include "EntityImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  struct LinkImpl_attached_joint
  {
	// Based on https://github.com/pal-robotics/gazebo_ros_link_attacher
	std::string model2;
	physics::ModelPtr m2;
	std::string link2;
	physics::LinkPtr l2;
	physics::JointPtr joint;
  };
  class LinkImpl : public virtual rrgz::Link_default_abstract_impl, public virtual EntityImpl
  {
  public:
	  LinkImpl(physics::LinkPtr l);
	 
      virtual RR::RRListPtr<RR::RRArray<char> > get_sensor_names() override;

	  virtual void attach_link(const std::string& model, const std::string& link_name) override;

	  virtual void attach_link_with_pose(const std::string& model, const std::string& link_name, const com::robotraconteur::geometry::Pose& pose) override;

	  virtual void detach_link(const std::string& model, const std::string& link_name) override;

	  virtual std::string RRType() { return Link_default_abstract_impl::RRType(); }

  protected:
  	  boost::weak_ptr<physics::Link> gz_link;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;

  	  physics::LinkPtr get_link();
  	  
  	  RR::RRListPtr<RR::RRArray<double > > applied_wrenches;
  	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  std::vector<LinkImpl_attached_joint> attached_link_joints;
	  bool get_attached_link_joint(std::string model2, std::string link2, LinkImpl_attached_joint &joint);
  };

}
