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
namespace geometry=com::robotraconteur::geometry;

namespace RobotRaconteurGazeboServerPlugin
{
  class JointImpl : public virtual rrgz::Joint_default_impl, public RR_ENABLE_SHARED_FROM_THIS<JointImpl>
  {
  public:
	  JointImpl(physics::JointPtr j);

	  virtual void Init();

	  static void OnUpdate(RR_WEAK_PTR<JointImpl> j, const common::UpdateInfo & _info);


	  virtual std::string get_Name() override; 
	  virtual std::string get_ScopedName() override;  

	  virtual std::string get_ParentLinkName() override;	  

	  virtual std::string get_ChildLinkName() override;	  

	  virtual uint32_t get_AxisCount() override;
	  
	  virtual RR::RRNamedArrayPtr<geometry::Vector3 > GetGlobalAxes() override;

	  virtual RR::RRNamedArrayPtr<geometry::Vector3> GetLocalAxes() override;

	  virtual void set_AxesPositions(RR::WirePtr<RR::RRArrayPtr<double> > value) override;
	  	  
	  virtual void set_AxesVelocities(RR::WirePtr<RR::RRArrayPtr<double> > value) override;
	  	  
	  virtual void set_AxesForce(RR::WirePtr<RR::RRArrayPtr<double> > value) override;
	  	  
	  virtual void SetAxisPosition(uint32_t axis, double Position);

	  virtual void SetAxisVelocity(uint32_t axis, double vel);

	  virtual void set_ForceTorque(RR::WirePtr<rrgz::JointWrench> value);

	 
  protected:

  	  std::string link_name;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;
  	  boost::weak_ptr<physics::Joint> gz_joint;

  	 physics::JointPtr get_joint();

  	 virtual void OnUpdate1(const common::UpdateInfo & _info);

  	 event::ConnectionPtr updateConnection;

  	 RR::RRArrayPtr<double> axes_forces;  	 	
  };

}
