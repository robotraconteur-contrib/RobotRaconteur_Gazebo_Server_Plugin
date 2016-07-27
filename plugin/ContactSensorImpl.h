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

#include "SensorImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class ContactSensorImpl : public virtual rrgz::ContactSensor, public virtual SensorImpl
  {
  public:
  	  ContactSensorImpl(sensors::ContactSensorPtr gz_contact);

  	  void Init();

  	  static void OnUpdate(RR_WEAK_PTR<SensorImpl> c);

  	  virtual std::string get_Name() {return SensorImpl::get_Name();}
  	  virtual void set_Name(std::string value) {SensorImpl::set_Name(value);}

  	  virtual std::string get_Type() {return SensorImpl::get_Type();}
  	  virtual void set_Type(std::string value) {SensorImpl::set_Type(value);}

  	  virtual std::string get_ParentName() {return SensorImpl::get_ParentName();}
	  virtual void set_ParentName(std::string value) {SensorImpl::set_ParentName(value);}

  	  virtual RR_SHARED_PTR<rrgz::Pose > get_Pose() {return SensorImpl::get_Pose();}
	  virtual void set_Pose(RR_SHARED_PTR<rrgz::Pose > value) {SensorImpl::set_Pose(value);}

  	  virtual uint8_t get_Active() {return SensorImpl::get_Active();}
  	  virtual void set_Active(uint8_t value) {SensorImpl::set_Active(value);}

  	  virtual double get_UpdateRate() {return SensorImpl::get_UpdateRate();}
  	  virtual void set_UpdateRate(double value) {SensorImpl::set_UpdateRate(value);}

  	  virtual double get_LastUpdateTime() {return SensorImpl::get_LastUpdateTime();}
  	  virtual void set_LastUpdateTime(double value) {SensorImpl::set_LastUpdateTime(value);}

  	  virtual double get_LastMeasurementTime() {return SensorImpl::get_LastMeasurementTime();}
  	  virtual void set_LastMeasurementTime(double value) {SensorImpl::set_LastMeasurementTime(value);}

  	  virtual RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > CaptureContacts();

  	  virtual RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > > get_ContactWire();
  	  virtual void set_ContactWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > > value);

      virtual std::string RRType() {return "experimental.gazebo.ContactSensor";  }
  protected:
      sensors::ContactSensorPtr get_contactsensor();

      void OnUpdate1();
      event::ConnectionPtr updateConnection;

      RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > > m_ContactWire;
      RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > > m_ContactWire_b;
  };


}
