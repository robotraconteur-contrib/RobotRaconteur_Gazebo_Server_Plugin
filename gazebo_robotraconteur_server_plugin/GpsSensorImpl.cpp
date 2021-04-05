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
	GpsSensorImpl::GpsSensorImpl(sensors::GpsSensorPtr gps) : SensorImpl(gps)
	{
		gz_gps = gps;
	}

	static gps::GpsStatePtr gz_to_rr_gpsstate(sensors::GpsSensorPtr& c)
	{		
		gps::GpsStatePtr o(new gps::GpsState());
		
		memset(o.get(),sizeof(gps::GpsState),0);

		o->altitude = c->Altitude();
		o->latitude_deg = c->Latitude().Degree();
		o->longitude_deg = c->Longitude().Degree();

		//TODO: Access velocities?
		/*o->velocity_east = 0.0;
		o->velocity_north = 0.0;
		o->velocity_up = 0.0;*/
		return o;
	}

	void GpsSensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		SensorImpl::RRServiceObjectInit(context, service_path);

		rr_downsampler->AddWireBroadcaster(rrvar_state);

		boost::weak_ptr<GpsSensorImpl> weak_this = RR::rr_cast<GpsSensorImpl>(shared_from_this());
		this->rrvar_state->GetWire()->SetPeekInValueCallback(
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
		SensorImpl::OnUpdate1();
		auto s = gz_gps.lock();
		if (!s) return;		
		auto i = gz_to_rr_gpsstate(s);
		rrvar_state->SetOutValue(i);		
	}

}
