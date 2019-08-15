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


	RR::RRArrayPtr<double> MagnetometerSensorImpl::get_MagneticField()
	{
		boost::mutex::scoped_lock lock(this_lock);
		sensors::MagnetometerSensorPtr c=get_magnetometersensor();
		auto v= c->MagneticField();
		auto o=RR::AllocateRRArray<double>(3);
		(*o)[0]=v[0];
		(*o)[1]=v[1];
		(*o)[2]=v[2];
		return o;
	}
	
	RR::WirePtr<RR::RRArrayPtr<double> > MagnetometerSensorImpl::get_MagneticFieldWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_MagneticFieldWire;
	}
	void MagnetometerSensorImpl::set_MagneticFieldWire(RR::WirePtr<RR::RRArrayPtr<double> > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		m_MagneticFieldWire=value;
		m_MagneticFieldWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR::RRArrayPtr<double> > >();
		m_MagneticFieldWire_b->Init(m_MagneticFieldWire);
	}

	sensors::MagnetometerSensorPtr MagnetometerSensorImpl::get_magnetometersensor()
	{
		return std::dynamic_pointer_cast<sensors::MagnetometerSensor>(get_sensor());
	}

	void MagnetometerSensorImpl::OnUpdate1()
	{
		RR::WireBroadcasterPtr<RR::RRArrayPtr<double> > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_MagneticFieldWire_b;
		}
		if (b)
		{
			auto i=get_MagneticField();
			b->SetOutValue(i);
		}
	}

}
