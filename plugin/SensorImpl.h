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
  class SensorImpl : public virtual rrgz::Sensor, public RR_ENABLE_SHARED_FROM_THIS<SensorImpl>
  {
  public:
	  SensorImpl(sensors::SensorPtr gz_sensor);

	  virtual std::string get_Name();
	  virtual void set_Name(std::string value);

	  virtual std::string get_Type();
	  virtual void set_Type(std::string value);

	  virtual std::string get_ParentName();
	  virtual void set_ParentName(std::string value);

	  virtual RR_SHARED_PTR<rrgz::Pose > get_Pose();
	  virtual void set_Pose(RR_SHARED_PTR<rrgz::Pose > value);

	  virtual uint8_t get_Active();
	  virtual void set_Active(uint8_t value);

	  virtual double get_UpdateRate();
	  virtual void set_UpdateRate(double value);

	  virtual double get_LastUpdateTime();
	  virtual void set_LastUpdateTime(double value);

	  virtual double get_LastMeasurementTime();
	  virtual void set_LastMeasurementTime(double value);

  protected:

	  std::weak_ptr<sensors::Sensor> gz_sensor;
	  sensors::SensorPtr get_sensor();
	  boost::mutex this_lock;
  };

}
