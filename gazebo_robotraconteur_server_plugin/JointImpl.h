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
  class JointImpl : public virtual rrgz::Joint, public RR_ENABLE_SHARED_FROM_THIS<JointImpl>
  {
  public:
	  JointImpl(physics::JointPtr j);

	  virtual void Init();

	  static void OnUpdate(RR_WEAK_PTR<JointImpl> j, const common::UpdateInfo & _info);


	  virtual std::string get_Name() override; 
	  virtual std::string get_ScopedName() override;  

	  virtual std::string get_ParentLinkName() override;	  

	  virtual std::string get_ChildLinkName() override;	  

	  virtual int32_t get_AxisCount() override;
	  
	  virtual RR::RRMapPtr<int32_t,RR::RRArray<double > > get_AxesAngles() override;
	  virtual void set_AxesAngles(RR::RRMapPtr<int32_t,RR::RRArray<double > > value) override;

	  virtual RR::RRMapPtr<int32_t,RR::RRArray<double > > get_AxesVelocities() override;
	  virtual void set_AxesVelocities(RR::RRMapPtr<int32_t,RR::RRArray<double > > value) override;

	  virtual RR::RRArrayPtr<double > GetGlobalAxis(int32_t axis) override;

	  virtual RR::RRArrayPtr<double > GetLocalAxis(int32_t axis) override;


	  virtual double GetAxisAngle(int32_t axis) override;

	  virtual void SetAxisPosition(int32_t axis, double value) override;

	  virtual double GetAxisVelocity(int32_t axis) override;

	  virtual void SetAxisVelocity(int32_t axis, double value) override;

	  virtual double GetForce(int32_t axis) override;

	  virtual void SetForce(int32_t axis, double value) override;

	  virtual RR::RRArrayPtr<double > GetForceTorque(int32_t link) override;

	  virtual RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double > > > get_AxisAngleGetWire() override;
	  virtual void set_AxisAngleGetWire(RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double > > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > get_AxisVelocityGetWire() override;
	  virtual void set_AxisVelocityGetWire(RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > get_ForceTorqueGetWire() override;
	  virtual void set_ForceTorqueGetWire(RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > get_AxisPositionSetWire() override;
	  virtual void set_AxisPositionSetWire(RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > get_AxisVelocitySetWire() override;
	  virtual void set_AxisVelocitySetWire(RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > value) override;

	  virtual RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > get_ForceSetWire() override;
	  virtual void set_ForceSetWire(RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > value) override;


  protected:

  	  std::string link_name;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;
  	  boost::weak_ptr<physics::Joint> gz_joint;

  	 physics::JointPtr get_joint();

  	 virtual void OnUpdate1(const common::UpdateInfo & _info);

  	  event::ConnectionPtr updateConnection;

  	  RR::RRArrayPtr<double> axes_forces;
  	  boost::mutex this_lock;

  	RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisAngleGetWire;
  	RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisVelocityGetWire;
  	RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_ForceTorqueGetWire;
  	RR::WireBroadcasterPtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisAngleGetWire_b;
  	RR::WireBroadcasterPtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisVelocityGetWire_b;
  	RR::WireBroadcasterPtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_ForceTorqueGetWire_b;

  	RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisPositionSetWire;
	RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisVelocitySetWire;
	RR::WirePtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_ForceSetWire;

	RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisPositionSetWire_conn;
	RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_AxisVelocitySetWire_conn;
	RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double >  > > m_ForceSetWire_conn;

	static void OnAxisPositionSetWireConnect(RR_WEAK_PTR<JointImpl> l, RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double > > > connection);
	static void OnAxisPositionSetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double> > > connection);
	static void OnAxisVelocitySetWireConnect(RR_WEAK_PTR<JointImpl> l, RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double> > > connection);
	static void OnAxisVelocitySetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double> > > connection);
	static void OnForceSetWireConnect(RR_WEAK_PTR<JointImpl> l, RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double> > > connection);
	static void OnForceSetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR::WireConnectionPtr<RR::RRMapPtr<int32_t,RR::RRArray<double> > > connection);
  };

}
