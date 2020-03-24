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

	static gps::GpsStatePtr gz_to_rr_gpsstate(sensors::GpsSensorPtr& c)
	{		
		gps::GpsStatePtr o(new gps::GpsState());
		o->altitude = c->Altitude();
		o->latitude_deg = c->Latitude().Degree();
		o->longitude_deg = c->Longitude().Degree();

		//TODO: Access velocities?
		o->velocity_east = 0.0;
		o->velocity_north = 0.0;
		o->velocity_up = 0.0;
		return o;
	}
			
	void GpsSensorImpl::set_State(RR::WirePtr<gps::GpsStatePtr> value)
	{
		GpsSensor_default_abstract_impl::set_State(value);				
		boost::weak_ptr<GpsSensorImpl> weak_this = RR::rr_cast<GpsSensorImpl>(shared_from_this());
		this->rrvar_State->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto s = this_->get_gpssensor();
				return gz_to_rr_gpsstate(s);
			}
		);
	}

	sensors::GpsSensorPtr GpsSensorImpl::get_gpssensor()
	{
		return std::dynamic_pointer_cast<sensors::GpsSensor>(get_sensor());
	}

	void GpsSensorImpl::OnUpdate1()
	{
		RR::WireBroadcasterPtr<gps::GpsStatePtr> b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b = rrvar_State;
		}
		if (b)
		{
			auto s = get_gpssensor();
			auto i = gz_to_rr_gpsstate(s);
			b->SetOutValue(i);
		}
	}

}
