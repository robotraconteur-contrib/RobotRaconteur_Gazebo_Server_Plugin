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

#include "AltimeterSensorImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	AltimeterSensorImpl::AltimeterSensorImpl(sensors::AltimeterSensorPtr gz_contact) : SensorImpl(gz_contact)
	{

	}

	void AltimeterSensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_altimetersensor()->ConnectUpdated(boost::bind(&AltimeterSensorImpl::OnUpdate,c));

		boost::weak_ptr<AltimeterSensorImpl> weak_this = RR::rr_cast<AltimeterSensorImpl>(shared_from_this());
		this->rrvar_altitude->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Sensor has been released");
				auto j = this_->get_altimetersensor();
				return j->Altitude();
			}
		);
	}

	void AltimeterSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<AltimeterSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<AltimeterSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}
		
	sensors::AltimeterSensorPtr AltimeterSensorImpl::get_altimetersensor()
	{
		return std::dynamic_pointer_cast<sensors::AltimeterSensor>(get_sensor());
	}

	void AltimeterSensorImpl::OnUpdate1()
	{
		rrvar_altitude->SetOutValue(get_altimetersensor()->Altitude());		
	}

}
