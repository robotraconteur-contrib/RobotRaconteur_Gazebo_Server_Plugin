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
#include "experimental__gazebo.h"
#include "experimental__gazebo_stubskel.h"

#pragma once

using namespace gazebo;
namespace RR=RobotRaconteur;
namespace rrgz=experimental::gazebo;

#include "SensorImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class AltimeterSensorImpl : public virtual rrgz::AltimeterSensor_default_abstract_impl, public virtual RR::IRRServiceObject, public virtual SensorImpl
  {
  public:
  	  AltimeterSensorImpl(sensors::AltimeterSensorPtr gz_altimeter);

  	  static void OnUpdate(RR_WEAK_PTR<SensorImpl> c);

  	  virtual std::string RRType() {return AltimeterSensor_default_abstract_impl::RRType();  }

      virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

  protected:
      sensors::AltimeterSensorPtr get_altimetersensor();

      void OnUpdate1();
      event::ConnectionPtr updateConnection;
  };


}
