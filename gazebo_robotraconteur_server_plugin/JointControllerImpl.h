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
class ModelImpl;
class JointControllerImpl : public virtual rrgz::JointController, public virtual RR_ENABLE_SHARED_FROM_THIS<JointControllerImpl>
{
public:

	  JointControllerImpl(RR_SHARED_PTR<ModelImpl> model, physics::ModelPtr gz_model);

	  virtual void Init();

	  virtual RR::RRListPtr<RR::RRArray<char> > get_JointNames() override;	  

	  virtual RR::RRMapPtr<std::string,rrgz::PIDParam> get_PositionPIDs() override;

	  virtual RR::RRMapPtr<std::string,rrgz::PIDParam> get_VelocityPIDs() override;	  

	  virtual RR::RRMapPtr<std::string,RR::RRArray<double >  > get_JointTargetPositions() override;
	  virtual void set_JointTargetPositions(RR::RRMapPtr<std::string,RR::RRArray<double > > value) override;

	  virtual RR::RRMapPtr<std::string,RR::RRArray<double > > get_JointTargetVelocities() override;
	  virtual void set_JointTargetVelocities(RR::RRMapPtr<std::string,RR::RRArray<double > > value) override;

	  virtual RR::RRMapPtr<std::string,RR::RRArray<double > > get_JointForces() override;
	  
	  virtual void AddJoint(const std::string& name) override;

	  virtual void SetPositionPID(const std::string& name, rrgz::PIDParamPtr pid) override;

	  virtual void SetVelocityPID(const std::string& name, rrgz::PIDParamPtr pid) override;

	  virtual RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > >  get_JointTargetPositionsSetWire() override;
	  virtual void set_JointTargetPositionsSetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > >  get_JointTargetVelocitiesSetWire() override;
	  virtual void set_JointTargetVelocitiesSetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > get_JointActualPositionsGetWire() override;
	  virtual void set_JointActualPositionsGetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > get_JointActualVelocitiesGetWire() override;
	  virtual void set_JointActualVelocitiesGetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > value) override;


protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  boost::shared_ptr<physics::JointController> gz_controller;
	  std::map<std::string,boost::weak_ptr<physics::Joint> > gz_joints;

	  boost::mutex this_lock;
	  virtual physics::ModelPtr get_model();

	  RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointActualPositionsGetWire;
	  RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointActualVelocitiesGetWire;
	  RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointActualPositionsGetWire_b;
	  RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointActualVelocitiesGetWire_b;

	  RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointTargetPositionsSetWire;
	  RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointTargetVelocitiesSetWire;

	  RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointTargetPositionsSetWire_c;
	  RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > m_JointTargetVelocitiesSetWire_c;

	  static void OnJointTargetPositionsSetWireConnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > connection);
	  static void OnJointTargetPositionsSetWireDisconnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > connection);

	  static void OnJointTargetVelocitiesSetWireConnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > >  connection);
	  static void OnJointTargetVelocitiesSetWireDisconnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double >  > > connection);

	  static void OnUpdate(RR_WEAK_PTR<JointControllerImpl> j, const common::UpdateInfo& _info);
	  void OnUpdate1(const common::UpdateInfo& _info);

	  event::ConnectionPtr updateConnection;

};

}
