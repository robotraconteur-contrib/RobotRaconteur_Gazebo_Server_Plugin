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
  class ForceTorqueSensorImpl : public virtual rrgz::ForceTorqueSensor_default_impl, public virtual SensorImpl
  {
  public:
  	  ForceTorqueSensorImpl(sensors::ForceTorqueSensorPtr gz_forcetorque);

  	  void Init();

  	  static void OnUpdate(RR_WEAK_PTR<SensorImpl> c);

  	  virtual std::string get_Name() override {return SensorImpl::get_Name();}	  

	  virtual std::string get_Type() override {return SensorImpl::get_Type();}	  

  	  virtual std::string get_ParentName() override {return SensorImpl::get_ParentName();}	  

	  virtual geometry::Pose get_Pose() override { return SensorImpl::get_Pose(); }

	  virtual RR::rr_bool get_Active() override { return SensorImpl::get_Active(); }
	  virtual void set_Active(RR::rr_bool value) override { SensorImpl::set_Active(value); }

	  virtual double get_UpdateRate() override { return SensorImpl::get_UpdateRate(); }
	  virtual void set_UpdateRate(double value) override { SensorImpl::set_UpdateRate(value); }

	  virtual datetime::Duration get_LastUpdateTime() override { return SensorImpl::get_LastUpdateTime(); }

	  virtual datetime::Duration get_LastMeasurementTime() override { return SensorImpl::get_LastMeasurementTime(); }
	    	  
  	  virtual void set_ForceTorque(RR::WirePtr<geometry::Wrench> value) override;

      virtual std::string RRType() {return "experimental.gazebo.ForceTorqueSensor";  }
  protected:
      sensors::ForceTorqueSensorPtr get_forcetorquesensor();

      void OnUpdate1();
      event::ConnectionPtr updateConnection;
  };


}
