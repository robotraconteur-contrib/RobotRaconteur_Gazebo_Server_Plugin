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

#include "EntityImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	LightImpl::LightImpl(rendering::LightPtr light)
	{
		this->light=light;
		this->gzNode = transport::NodePtr(new transport::Node());
		this->gzNode->Init();
		this->lightPub = this->gzNode->Advertise<msgs::Light>("~/light/modify");
	}

	std::string LightImpl::get_name()
	{
		return get_light()->Name();
	}

	
	std::string LightImpl::get_type()
	{
		return get_light()->Type();
	}
	
	geometry::Pose LightImpl::get_pose()
	{
		auto pos = get_light()->Position();
		auto rot = get_light()->Rotation();
		geometry::Pose o;

		o.s.position.s.x = pos.X();
		o.s.position.s.y = pos.Y();
		o.s.position.s.z = pos.Z();

		o.s.orientation.s.w = rot.W();
		o.s.orientation.s.x = rot.X();
		o.s.orientation.s.y = rot.Y();
		o.s.orientation.s.z = rot.Z();

		return o;
	}
	
	geometry::Vector3 LightImpl::get_direction()
	{
		auto d=get_light()->Direction();
		geometry::Vector3 o;
		o.s.x = d.X();
		o.s.y = d.Y();
		o.s.z = d.Z();
		return o;
	}
	
	color::ColorRGBAf LightImpl::get_diffuse_color()
	{
		auto c=get_light()->DiffuseColor();
		color::ColorRGBAf o;
		o.s.a = c.A();
		o.s.b = c.B();
		o.s.g = c.G();
		o.s.r = c.R();
		return o;
	}
	void LightImpl::set_diffuse_color(const color::ColorRGBAf& value)
	{
		ignition::math::Color c(value.s.r, value.s.g, value.s.b, value.s.a);
		auto l=get_light();
		l->SetDiffuseColor(c);

		// Publish updated color so gzclient updates
		msgs::Light lightMsg;
		l->FillMsg(lightMsg);
		this->lightPub->Publish(lightMsg);

	}

	color::ColorRGBAf LightImpl::get_specular_color()
	{
		auto c=get_light()->SpecularColor();
		color::ColorRGBAf o;
		o.s.a = c.A();
		o.s.b = c.B();
		o.s.g = c.G();
		o.s.r = c.R();
		return o;
	}
	void LightImpl::set_specular_color(const color::ColorRGBAf& value)
	{
		ignition::math::Color c(value.s.r, value.s.g, value.s.b, value.s.a);
		auto l=get_light();
		l->SetSpecularColor(c);

		// Publish updated color so gzclient updates
		msgs::Light lightMsg;
		l->FillMsg(lightMsg);
		this->lightPub->Publish(lightMsg);
	}

	rendering::LightPtr LightImpl::get_light()
	{
		rendering::LightPtr l=light.lock();
		if(!l) throw std::runtime_error("Light has been released");
		return l;
	}

}
