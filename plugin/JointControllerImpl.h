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

	  virtual RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > get_JointNames();
	  virtual void set_JointNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value);

	  virtual RR_SHARED_PTR<RR::RRMap<std::string,rrgz::PIDParam  > > get_PositionPIDs();
	  virtual void set_PositionPIDs(RR_SHARED_PTR<RR::RRMap<std::string,rrgz::PIDParam  > > value);

	  virtual RR_SHARED_PTR<RR::RRMap<std::string,rrgz::PIDParam  > > get_VelocityPIDs();
	  virtual void set_VelocityPIDs(RR_SHARED_PTR<RR::RRMap<std::string,rrgz::PIDParam  > > value);

	  virtual RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > get_JointTargetPositions();
	  virtual void set_JointTargetPositions(RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > value);

	  virtual RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > get_JointTargetVelocities();
	  virtual void set_JointTargetVelocities(RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > value);

	  virtual RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > get_JointForces();
	  virtual void set_JointForces(RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > value);

	  virtual void AddJoint(std::string name);

	  virtual void SetPositionPID(std::string name, RR_SHARED_PTR<rrgz::PIDParam > pid);

	  virtual void SetVelocityPID(std::string name, RR_SHARED_PTR<rrgz::PIDParam > pid);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > get_JointTargetPositionsSetWire();
	  virtual void set_JointTargetPositionsSetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > get_JointTargetVelocitiesSetWire();
	  virtual void set_JointTargetVelocitiesSetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > get_JointActualPositionsGetWire();
	  virtual void set_JointActualPositionsGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > get_JointActualVelocitiesGetWire();
	  virtual void set_JointActualVelocitiesGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > value);


protected:
	  boost::weak_ptr<physics::Model> gz_model;
	  boost::shared_ptr<physics::JointController> gz_controller;
	  std::map<std::string,boost::weak_ptr<physics::Joint> > gz_joints;

	  boost::mutex this_lock;
	  virtual physics::ModelPtr get_model();

	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointActualPositionsGetWire;
	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointActualVelocitiesGetWire;
	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointActualPositionsGetWire_b;
	  RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointActualVelocitiesGetWire_b;

	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointTargetPositionsSetWire;
	  RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointTargetVelocitiesSetWire;

	  RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointTargetPositionsSetWire_c;
	  RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > m_JointTargetVelocitiesSetWire_c;

	  static void OnJointTargetPositionsSetWireConnect(RR_WEAK_PTR<JointControllerImpl> c, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > connection);
	  static void OnJointTargetPositionsSetWireDisconnect(RR_WEAK_PTR<JointControllerImpl> c, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > connection);

	  static void OnJointTargetVelocitiesSetWireConnect(RR_WEAK_PTR<JointControllerImpl> c, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > connection);
	  static void OnJointTargetVelocitiesSetWireDisconnect(RR_WEAK_PTR<JointControllerImpl> c, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<std::string,RR::RRArray<double >  > > > > connection);

	  static void OnUpdate(RR_WEAK_PTR<JointControllerImpl> j, const common::UpdateInfo& _info);
	  void OnUpdate1(const common::UpdateInfo& _info);

	  event::ConnectionPtr updateConnection;

};

}
