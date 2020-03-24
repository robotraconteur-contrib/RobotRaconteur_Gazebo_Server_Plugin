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

#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	MagnetometerSensorImpl::MagnetometerSensorImpl(sensors::MagnetometerSensorPtr gz_contact) : SensorImpl(gz_contact)
	{

	}

	void MagnetometerSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_magnetometersensor()->ConnectUpdated(boost::bind(&MagnetometerSensorImpl::OnUpdate,c));
	}

	void MagnetometerSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<MagnetometerSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<MagnetometerSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}
	
	void MagnetometerSensorImpl::set_MagneticField(RR::WirePtr<geometry::Vector3> value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		MagnetometerSensor_default_abstract_impl::set_MagneticField(value);
		boost::weak_ptr<MagnetometerSensorImpl> weak_this = RR::rr_cast<MagnetometerSensorImpl>(shared_from_this());
		this->rrvar_MagneticField->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Sensor has been released");
				auto j = this_->get_magnetometersensor();
				auto v = j->MagneticField();
				geometry::Vector3 o;
				o.s.x = v.X();
				o.s.y = v.Y();
				o.s.y = v.Z();
				return o;
			}
		);
	}

	sensors::MagnetometerSensorPtr MagnetometerSensorImpl::get_magnetometersensor()
	{
		return std::dynamic_pointer_cast<sensors::MagnetometerSensor>(get_sensor());
	}

	void MagnetometerSensorImpl::OnUpdate1()
	{
		RR::WireBroadcasterPtr<geometry::Vector3> b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=rrvar_MagneticField;
		}
		if (b)
		{
			auto j = get_magnetometersensor();
			auto v = j->MagneticField();
			geometry::Vector3 o;
			o.s.x = v.X();
			o.s.y = v.Y();
			o.s.y = v.Z();
			b->SetOutValue(o);
		}
	}
}
