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

#include "RaySensorImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	AltimeterSensorImpl::AltimeterSensorImpl(sensors::AltimeterSensorPtr gz_contact) : SensorImpl(gz_contact)
	{

	}

	void AltimeterSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_altimetersensor()->ConnectUpdated(boost::bind(&AltimeterSensorImpl::OnUpdate,c));
	}

	void AltimeterSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<AltimeterSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<AltimeterSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}


	double AltimeterSensorImpl::get_Altitude()
	{
		boost::mutex::scoped_lock lock(this_lock);
		sensors::AltimeterSensorPtr c=get_altimetersensor();
		return c->Altitude();
	}
	void AltimeterSensorImpl::set_Altitude(double value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::Wire<double> > AltimeterSensorImpl::get_AltitudeWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_AltitudeWire;
	}
	void AltimeterSensorImpl::set_AltitudeWire(RR_SHARED_PTR<RR::Wire<double> > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		m_AltitudeWire=value;
		m_AltitudeWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<double> >();
		m_AltitudeWire_b->Init(m_AltitudeWire);
	}

	sensors::AltimeterSensorPtr AltimeterSensorImpl::get_altimetersensor()
	{
		return std::dynamic_pointer_cast<sensors::AltimeterSensor>(get_sensor());
	}

	void AltimeterSensorImpl::OnUpdate1()
	{
		RR_SHARED_PTR<RR::WireBroadcaster<double> > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_AltitudeWire_b;
		}
		if (b)
		{
			auto i=get_Altitude();
			b->SetOutValue(i);
		}
	}

}
