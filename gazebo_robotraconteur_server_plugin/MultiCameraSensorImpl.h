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
namespace image = com::robotraconteur::image;

#include "SensorImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class MultiCameraSensorImpl : public virtual rrgz::MultiCameraSensor_default_abstract_impl, public virtual RR::IRRServiceObject, public virtual SensorImpl
  {
  public:
	  MultiCameraSensorImpl(sensors::MultiCameraSensorPtr gz_camera);

	  static void OnUpdate(RR_WEAK_PTR<SensorImpl> c);

	  virtual int32_t get_camera_count() override;	  

	  virtual image::ImagePtr capture_image(int32_t ind) override;
	  	  
	  virtual std::string RRType() {return rrgz::MultiCameraSensor_default_abstract_impl::RRType();  }

	  virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

  protected:
	  sensors::MultiCameraSensorPtr get_camera();
	  
	  void OnUpdate1();

	  event::ConnectionPtr updateConnection;
  };

}
