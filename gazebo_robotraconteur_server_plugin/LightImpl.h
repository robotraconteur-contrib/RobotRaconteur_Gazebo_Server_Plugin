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

#include "experimental__gazebo.h"
#include "experimental__gazebo_stubskel.h"

#pragma once

using namespace gazebo;
namespace RR=RobotRaconteur;
namespace rrgz=experimental::gazebo;

namespace RobotRaconteurGazeboServerPlugin
{
class LightImpl : public virtual rrgz::Light, public RR_ENABLE_SHARED_FROM_THIS<LightImpl>
{
public:
	LightImpl(rendering::LightPtr light);

	virtual void Init() {};

	virtual std::string get_Name() override;	

	virtual std::string get_Type() override;	

	virtual rrgz::PosePtr get_Pose() override;
	
	virtual RR::RRArrayPtr<double > get_Direction() override;
	
	virtual rrgz::ColorPtr get_DiffuseColor() override;
	virtual void set_DiffuseColor(rrgz::ColorPtr value) override;

	virtual rrgz::ColorPtr get_SpecularColor() override;
	virtual void set_SpecularColor(rrgz::ColorPtr value) override;

protected:
	boost::weak_ptr<rendering::Light> light;

	rendering::LightPtr get_light();


};
}
