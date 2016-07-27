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
	RaySensorImpl::RaySensorImpl(sensors::RaySensorPtr gz_RaySensor) : SensorImpl(gz_RaySensor)
	{

	}

	void RaySensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_raysensor()->ConnectUpdated(boost::bind(&RaySensorImpl::OnUpdate,c));
	}

	RR_SHARED_PTR<rrgz::LaserScan > RaySensorImpl::CaptureScan()
	{

		boost::mutex::scoped_lock lock(this_lock);
		auto o=RR_MAKE_SHARED<rrgz::LaserScan>();
		sensors::RaySensorPtr c=get_raysensor();

		bool a=c->IsActive();
		c->SetActive(false);

		o->angleMax=c->AngleMax().Radian();
		o->angleMin=c->AngleMin().Radian();
		o->angleStep=c->AngleResolution();
		o->count=c->RayCount();
		o->verticalAngleMax=c->VerticalAngleMax().Radian();
		o->verticalAngleMin=c->VerticalAngleMin().Radian();
		o->verticalAngleStep=c->VerticalAngleResolution();
		o->verticalCount=c->VerticalRayCount();
		o->rangeResolution=c->RangeResolution();

		size_t sample_count=o->count*o->verticalCount;
		o->intensities=RR::AllocateRRArray<double>(sample_count);
		o->ranges=RR::AllocateRRArray<double>(sample_count);
		o->fiducial=RR::AllocateRRArray<int32_t>(sample_count);

		auto i=o->intensities->ptr();
		auto r=o->ranges->ptr();
		auto f=o->fiducial->ptr();
		for (uint32_t j=0; j<sample_count; j++)
		{
			i[j]=c->Retro(j);
			r[j]=c->Range(j);
			f[j]=c->Fiducial(j);
		}

		c->SetActive(a);

		return o;
	}

	sensors::RaySensorPtr RaySensorImpl::get_raysensor()
	{
		return std::dynamic_pointer_cast<sensors::RaySensor>(get_sensor());
	}

	RR_SHARED_PTR<RR::Pipe<RR_SHARED_PTR<rrgz::LaserScan > > > RaySensorImpl::get_ScanStream()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_ImageStream;
	}
	void RaySensorImpl::set_ScanStream(RR_SHARED_PTR<RR::Pipe<RR_SHARED_PTR<rrgz::LaserScan > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_ImageStream) throw std::runtime_error("Already set");
		m_ImageStream=value;
		m_ImageStream_b=RR_MAKE_SHARED<RR::PipeBroadcaster<RR_SHARED_PTR<rrgz::LaserScan > > >();
		m_ImageStream_b->Init(m_ImageStream,3);
	}

	void RaySensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<RaySensorImpl> c1=RR_DYNAMIC_POINTER_CAST<RaySensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	void RaySensorImpl::OnUpdate1()
	{

		RR_SHARED_PTR<RR::PipeBroadcaster<RR_SHARED_PTR<rrgz::LaserScan > > > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_ImageStream_b;
		}
		if (b)
		{

			auto i=CaptureScan();
			b->AsyncSendPacket(i, []() {});
		}

	}
}
