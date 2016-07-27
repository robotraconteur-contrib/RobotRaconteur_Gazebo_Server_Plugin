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
	GpsSensorImpl::GpsSensorImpl(sensors::GpsSensorPtr gz_contact) : SensorImpl(gz_contact)
	{

	}

	void GpsSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_gpssensor()->ConnectUpdated(boost::bind(&GpsSensorImpl::OnUpdate,c));
	}

	void GpsSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<GpsSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<GpsSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}


	RR_SHARED_PTR<rrgz::GpsState > GpsSensorImpl::get_ReadState()
	{
		boost::mutex::scoped_lock lock(this_lock);
		sensors::GpsSensorPtr c=get_gpssensor();
		auto o=RR_MAKE_SHARED<rrgz::GpsState>();
		o->altitude=c->Altitude();
		o->latitude_deg=c->Latitude().Degree();
		o->longitude_deg=c->Longitude().Degree();

		//TODO: Access velocities?
		o->velocity_east=0.0;
		o->velocity_north=0.0;
		o->velocity_up=0.0;
		return o;
	}
	void GpsSensorImpl::set_ReadState(RR_SHARED_PTR<rrgz::GpsState> value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::GpsState > > > GpsSensorImpl::get_StateWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_StateWire;
	}
	void GpsSensorImpl::set_StateWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::GpsState > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		m_StateWire=value;
		m_StateWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::GpsState> > >();
		m_StateWire_b->Init(m_StateWire);
	}

	sensors::GpsSensorPtr GpsSensorImpl::get_gpssensor()
	{
		return std::dynamic_pointer_cast<sensors::GpsSensor>(get_sensor());
	}

	void GpsSensorImpl::OnUpdate1()
	{
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::GpsState > > > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_StateWire_b;
		}
		if (b)
		{
			auto i=get_ReadState();
			b->SetOutValue(i);
		}
	}

}
