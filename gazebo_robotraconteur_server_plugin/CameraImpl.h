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
namespace geometry = com::robotraconteur::geometry;
namespace datetime = com::robotraconteur::datetime;
namespace image = com::robotraconteur::image;

#include "SensorImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class CameraImpl : public virtual rrgz::CameraSensor_default_abstract_impl, public virtual SensorImpl
  {
  public:
	  CameraImpl(sensors::CameraSensorPtr gz_camera);

	  void Init();

	  static void OnUpdate(RR_WEAK_PTR<SensorImpl> c);

	  virtual image::ImagePtr CaptureImage() override;
	  
	  virtual void set_ImageStream(RR::PipePtr<image::ImagePtr> value) override;

	  virtual std::string RRType() {return rrgz::CameraSensor_default_abstract_impl::RRType();  }
  protected:
	  sensors::CameraSensorPtr get_camera();	  

	  void OnUpdate1();

	  event::ConnectionPtr updateConnection;
  };

  namespace detail
  {
	  image::ImageEncoding::ImageEncoding gz_image_enconding_to_rr_encoding(const std::string& format);
  }
}
