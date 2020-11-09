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

#include "SensorImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	SensorImpl::SensorImpl(sensors::SensorPtr gz_sensor)
	{
		this->gz_sensor=gz_sensor;
	}

	std::string SensorImpl::get_name()
	{
		return get_sensor()->ScopedName();
	}
	
	std::string SensorImpl::get_type()
	{
		return get_sensor()->Type();
	}
	
	std::string SensorImpl::get_parent_name()
	{
		return get_sensor()->ParentName();
	}
	
	geometry::Pose SensorImpl::get_pose()
	{
		auto p=get_sensor()->Pose();
		geometry::Pose o;

		o.s.position.s.x = p.Pos().X();
		o.s.position.s.y = p.Pos().Y();
		o.s.position.s.z = p.Pos().Z();

		o.s.orientation.s.w = p.Rot().W();
		o.s.orientation.s.x = p.Rot().X();
		o.s.orientation.s.y = p.Rot().Y();
		o.s.orientation.s.z = p.Rot().Z();

		return o;
	}
		
	sensors::SensorPtr SensorImpl::get_sensor()
	{
		sensors::SensorPtr s=gz_sensor.lock();
		if (!s) throw std::runtime_error("Sensor has been released");
		return s;
	}

	RR::rr_bool SensorImpl::get_active()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return get_sensor()->IsActive() ? 1 : 0;
	}
	void SensorImpl::set_active(RR::rr_bool value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		get_sensor()->SetActive(value!=0);
	}

	double SensorImpl::get_update_rate()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return get_sensor()->UpdateRate();
	}
	void SensorImpl::set_update_rate(double value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		get_sensor()->SetUpdateRate(value);
	}

	datetime::Duration SensorImpl::get_last_update_time()
	{
		auto t1 = get_sensor()->LastUpdateTime();

		datetime::Duration o;
		o.clock_info.clock_type = datetime::ClockTypeCode::sim_clock_scaled;
		o.nanoseconds = t1.nsec;
		o.seconds = t1.sec;
		return o;
	}
	
	datetime::Duration SensorImpl::get_last_measurement_time()
	{
		auto t1 = get_sensor()->LastMeasurementTime();
		datetime::Duration o;
		o.clock_info.clock_type = datetime::ClockTypeCode::sim_clock_scaled;
		o.nanoseconds = t1.nsec;
		o.seconds = t1.sec;
		return o;
	}

	void SensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		rr_context = context;
		rr_path = service_path;

		rr_downsampler = boost::make_shared<RR::BroadcastDownsampler>();
		rr_downsampler->Init(context.lock(),0);

		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_sensor()->ConnectUpdated(boost::bind(&SensorImpl::OnUpdate,c));		
	}

	void SensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> e)
	{
		RR_SHARED_PTR<SensorImpl> e1=e.lock();
		if (!e1) return;
		e1->OnUpdate0();
	}

	void SensorImpl::OnUpdate0()
	{
		RR::BroadcastDownsamplerStep step(rr_downsampler);

		sensors::SensorPtr e = gz_sensor.lock();
		if (!e) 
		{
			auto context = rr_context.lock();
			if (context)
			{
				try
				{
				context->ReleaseServicePath(rr_path);
				}
				catch (std::exception&) {}
			}
			
			return;
		}

		OnUpdate1();
	}

	void SensorImpl::OnUpdate1()
	{

	}

	com::robotraconteur::device::isoch::IsochInfoPtr SensorImpl::get_isoch_info()
	{
		com::robotraconteur::device::isoch::IsochInfoPtr ret(new com::robotraconteur::device::isoch::IsochInfo());
		auto sensor = get_sensor();
		auto world = physics::get_world(sensor->WorldName());
		common::Time start_time = world->StartTime();
		ret->isoch_epoch.seconds = start_time.sec;
		ret->isoch_epoch.nanoseconds = start_time.nsec;
		ret->max_downsample = 100;
		ret->update_rate = sensor->UpdateRate();

		return ret;
	}

	uint32_t SensorImpl::get_isoch_downsample()
	{
		return rr_downsampler->GetClientDownsample(RR::ServerEndpoint::GetCurrentEndpoint()->GetLocalEndpoint());
	}

	void SensorImpl::set_isoch_downsample(uint32_t value)
	{
		return rr_downsampler->SetClientDownsample(RR::ServerEndpoint::GetCurrentEndpoint()->GetLocalEndpoint(), value);
	}
}
