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

#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	MagnetometerSensorImpl::MagnetometerSensorImpl(sensors::MagnetometerSensorPtr mag) : SensorImpl(mag)
	{
		gz_mag = mag;
	}

	void MagnetometerSensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		SensorImpl::RRServiceObjectInit(context, service_path);

		rr_downsampler->AddWireBroadcaster(rrvar_magnetic_field);

		boost::weak_ptr<MagnetometerSensorImpl> weak_this = RR::rr_cast<MagnetometerSensorImpl>(shared_from_this());
		this->rrvar_magnetic_field->GetWire()->SetPeekInValueCallback(
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
		SensorImpl::OnUpdate1();
		auto j = gz_mag.lock();
		if (!j) return;
		auto v = j->MagneticField();
		geometry::Vector3 o;
		o.s.x = v.X();
		o.s.y = v.Y();
		o.s.y = v.Z();
		rrvar_magnetic_field->SetOutValue(o);		
	}
}
