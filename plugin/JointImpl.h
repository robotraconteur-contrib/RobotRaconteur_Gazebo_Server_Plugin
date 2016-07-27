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


	  virtual std::string get_Name();
	  virtual void set_Name(std::string value);
	  virtual std::string get_ScopedName();
	  virtual void set_ScopedName(std::string value);

	  virtual std::string get_ParentLinkName();
	  virtual void set_ParentLinkName(std::string value);

	  virtual std::string get_ChildLinkName();
	  virtual void set_ChildLinkName(std::string value);

	  virtual int32_t get_AxisCount();
	  virtual void set_AxisCount(int32_t value);

	  virtual RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > get_AxesAngles();
	  virtual void set_AxesAngles(RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > value);

	  virtual RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > get_AxesVelocities();
	  virtual void set_AxesVelocities(RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > value);

	  virtual RR_SHARED_PTR<RR::RRArray<double > > GetGlobalAxis(int32_t axis);

	  virtual RR_SHARED_PTR<RR::RRArray<double > > GetLocalAxis(int32_t axis);


	  virtual double GetAxisAngle(int32_t axis);

	  virtual void SetAxisPosition(int32_t axis, double value);

	  virtual double GetAxisVelocity(int32_t axis);

	  virtual void SetAxisVelocity(int32_t axis, double value);

	  virtual double GetForce(int32_t axis);

	  virtual void SetForce(int32_t axis, double value);

	  virtual RR_SHARED_PTR<RR::RRArray<double > > GetForceTorque(int32_t link);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > get_AxisAngleGetWire();
	  virtual void set_AxisAngleGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > get_AxisVelocityGetWire();
	  virtual void set_AxisVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > get_ForceTorqueGetWire();
	  virtual void set_ForceTorqueGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > get_AxisPositionSetWire();
	  virtual void set_AxisPositionSetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > get_AxisVelocitySetWire();
	  virtual void set_AxisVelocitySetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value);

	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > get_ForceSetWire();
	  virtual void set_ForceSetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value);


  protected:

  	  std::string link_name;
  	  std::string model_name;
  	  physics::WorldPtr gz_world;
  	  boost::weak_ptr<physics::Joint> gz_joint;

  	 physics::JointPtr get_joint();

  	 virtual void OnUpdate1(const common::UpdateInfo & _info);

  	  event::ConnectionPtr updateConnection;

  	  RR_SHARED_PTR<RR::RRArray<double> > axes_forces;
  	  boost::mutex this_lock;

  	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisAngleGetWire;
  	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisVelocityGetWire;
  	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_ForceTorqueGetWire;
  	RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisAngleGetWire_b;
  	RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisVelocityGetWire_b;
  	RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_ForceTorqueGetWire_b;

  	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisPositionSetWire;
	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisVelocitySetWire;
	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_ForceSetWire;

	RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisPositionSetWire_conn;
	RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_AxisVelocitySetWire_conn;
	RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > m_ForceSetWire_conn;

	static void OnAxisPositionSetWireConnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection);
	static void OnAxisPositionSetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection);
	static void OnAxisVelocitySetWireConnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection);
	static void OnAxisVelocitySetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection);
	static void OnForceSetWireConnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection);
	static void OnForceSetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection);
  };

}
