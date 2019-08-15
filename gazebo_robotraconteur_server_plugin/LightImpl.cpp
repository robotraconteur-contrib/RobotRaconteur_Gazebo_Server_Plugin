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

#include "EntityImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	LightImpl::LightImpl(rendering::LightPtr light)
	{
		this->light=light;
	}

	std::string LightImpl::get_Name()
	{
		return get_light()->Name();
	}

	
	std::string LightImpl::get_Type()
	{
		return get_light()->Type();
	}
	
	rrgz::PosePtr LightImpl::get_Pose()
	{
		auto p=get_light()->Position();
		auto o_p=get_light()->Rotation();
		rrgz::PosePtr o(new rrgz::Pose());
		o->Position=RR::AllocateRRArray<double>(3);
		o->Orientation=RR::AllocateRRArray<double>(4);

		for (uint32_t i=0; i<3; i++) (*o->Position)[i]=p[i];
		(*o->Orientation)[0]=o_p.W();
		(*o->Orientation)[1]=o_p.X();
		(*o->Orientation)[2]=o_p.Y();
		(*o->Orientation)[3]=o_p.Z();
		return o;
	}
	
	RobotRaconteur::RRArrayPtr<double> LightImpl::get_Direction()
	{
		auto d=get_light()->Direction();
		auto o=RR::AllocateRRArray<double>(3);
		(*o)[0]=d[0];
		(*o)[1]=d[1];
		(*o)[2]=d[2];
		return o;
	}
	
	rrgz::ColorPtr LightImpl::get_DiffuseColor()
	{
		auto c=get_light()->DiffuseColor();
		rrgz::ColorPtr o(new rrgz::Color());
		o->a=c.A();
		o->b=c.B();
		o->g=c.G();
		o->r=c.R();
		return o;
	}
	void LightImpl::set_DiffuseColor(rrgz::ColorPtr value)
	{
		//TODO: This update doesn't apply to the gzclient viewer.
		//It does work with the cameras sensors.
		RR_NULL_CHECK(value);
		ignition::math::Color c(value->r, value->g, value->b, value->a);
		auto l=get_light();
		l->SetDiffuseColor(c);
	}

	rrgz::ColorPtr LightImpl::get_SpecularColor()
	{
		ignition::math::Color c=get_light()->SpecularColor();
		rrgz::ColorPtr o(new rrgz::Color());
		o->a=c.A();
		o->b=c.B();
		o->g=c.G();
		o->r=c.R();
		return o;
	}
	void LightImpl::set_SpecularColor(rrgz::ColorPtr value)
	{
		//TODO: This update doesn't apply to the gzclient viewer.
		//It does work with the cameras sensors.
		RR_NULL_CHECK(value);
		ignition::math::Color c(value->r, value->g, value->b, value->a);
		auto l=get_light();
		l->SetSpecularColor(c);
	}

	rendering::LightPtr LightImpl::get_light()
	{
		rendering::LightPtr l=light.lock();
		if(!l) throw std::runtime_error("Light has been released");
		return l;
	}

}
