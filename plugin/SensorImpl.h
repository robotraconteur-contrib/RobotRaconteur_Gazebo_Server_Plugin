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

	  virtual std::string get_Name() override;	  

	  virtual std::string get_Type() override;	  

	  virtual std::string get_ParentName() override;	  

	  virtual rrgz::PosePtr get_Pose() override;	  

	  virtual uint8_t get_Active() override;
	  virtual void set_Active(uint8_t value) override;

	  virtual double get_UpdateRate() override;
	  virtual void set_UpdateRate(double value) override;

	  virtual double get_LastUpdateTime() override;  

	  virtual double get_LastMeasurementTime() override;	  

  protected:

	  std::weak_ptr<sensors::Sensor> gz_sensor;
	  sensors::SensorPtr get_sensor();
	  boost::mutex this_lock;
  };

}
