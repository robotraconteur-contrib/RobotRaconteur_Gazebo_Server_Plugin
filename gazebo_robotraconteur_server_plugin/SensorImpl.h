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
#include "org__gazebosim__gazebo.h"
#include "org__gazebosim__gazebo_stubskel.h"

#pragma once

using namespace gazebo;
namespace RR=RobotRaconteur;
namespace rrgz=org::gazebosim::gazebo;
namespace geometry = com::robotraconteur::geometry;
namespace datetime = com::robotraconteur::datetime;

namespace RobotRaconteurGazeboServerPlugin
{
  class SensorImpl : public virtual rrgz::Sensor_default_impl, public virtual RR::IRRServiceObject, public RR_ENABLE_SHARED_FROM_THIS<SensorImpl>
  {
  public:
	  SensorImpl(sensors::SensorPtr gz_sensor);

	  virtual std::string get_name() override;	  

	  virtual std::string get_type() override;	  

	  virtual std::string get_parent_name() override;	  

	  virtual geometry::Pose get_pose() override;	  

	  virtual RR::rr_bool get_active() override;
	  virtual void set_active(RR::rr_bool value) override;

	  virtual double get_update_rate() override;
	  virtual void set_update_rate(double value) override;

	  virtual datetime::Duration get_last_update_time() override;  

	  virtual datetime::Duration get_last_measurement_time() override;

	  static void OnUpdate(RR_WEAK_PTR<SensorImpl> c);

	  virtual com::robotraconteur::device::isoch::IsochInfoPtr get_isoch_info() override;

	  virtual uint32_t get_isoch_downsample() override;

	  virtual void set_isoch_downsample(uint32_t value) override;

	  virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

  protected:

	  virtual void OnUpdate1();
	  virtual void OnUpdate0();

	  std::weak_ptr<sensors::Sensor> gz_sensor;
	  sensors::SensorPtr get_sensor();

	  event::ConnectionPtr updateConnection;

	  RR_WEAK_PTR<RR::ServerContext> rr_context;
	  std::string rr_path;

	  RR::BroadcastDownsamplerPtr rr_downsampler;
  };

}
