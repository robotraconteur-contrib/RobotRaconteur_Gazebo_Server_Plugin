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
namespace geometry = com::robotraconteur::geometry;
namespace color = com::robotraconteur::color;

namespace RobotRaconteurGazeboServerPlugin
{
class LightImpl : public virtual rrgz::Light, public RR_ENABLE_SHARED_FROM_THIS<LightImpl>
{
public:
	LightImpl(rendering::LightPtr light);

	virtual void Init() {};

	virtual std::string get_Name() override;	

	virtual std::string get_Type() override;	

	virtual geometry::Pose get_Pose() override;
	
	virtual geometry::Vector3 get_Direction() override;
	
	virtual color::ColorRGBAf get_DiffuseColor() override;
	virtual void set_DiffuseColor(const color::ColorRGBAf& value) override;

	virtual color::ColorRGBAf get_SpecularColor() override;
	virtual void set_SpecularColor(const color::ColorRGBAf& value) override;

protected:
	boost::weak_ptr<rendering::Light> light;

	rendering::LightPtr get_light();


};
}