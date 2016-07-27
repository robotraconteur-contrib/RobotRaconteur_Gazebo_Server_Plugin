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

	virtual std::string get_Name();
	virtual void set_Name(std::string value);

	virtual std::string get_Type();
	virtual void set_Type(std::string value);

	virtual RR_SHARED_PTR<rrgz::Pose > get_Pose();
	virtual void set_Pose(RR_SHARED_PTR<rrgz::Pose > value);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > get_Direction();
	virtual void set_Direction(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > value);

	virtual RR_SHARED_PTR<rrgz::Color > get_DiffuseColor();
	virtual void set_DiffuseColor(RR_SHARED_PTR<rrgz::Color > value);

	virtual RR_SHARED_PTR<rrgz::Color > get_SpecularColor();
	virtual void set_SpecularColor(RR_SHARED_PTR<rrgz::Color > value);

protected:
	boost::weak_ptr<rendering::Light> light;

	rendering::LightPtr get_light();


};
}
