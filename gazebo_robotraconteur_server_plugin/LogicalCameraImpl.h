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
namespace geometry = com::robotraconteur::geometry;
namespace datetime = com::robotraconteur::datetime;
namespace image = com::robotraconteur::image;
namespace objrec = com::robotraconteur::objectrecognition;

#include "SensorImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
  class LogicalCameraImpl : public virtual rrgz::LogicalCameraSensor_default_abstract_impl, public virtual SensorImpl
  {
  public:
	  LogicalCameraImpl(sensors::LogicalCameraSensorPtr gz_camera);

	  virtual objrec::RecognizedObjectsPtr capture_image() override;
	  
	  virtual std::string RRType() {return rrgz::LogicalCameraSensor_default_abstract_impl::RRType();  }

	  virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

  protected:
	  sensors::LogicalCameraSensorPtr get_camera();	  

	  virtual void OnUpdate1();

	 std::weak_ptr<sensors::LogicalCameraSensor> gz_camera;
  };

}
