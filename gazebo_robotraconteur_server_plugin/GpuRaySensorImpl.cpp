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
	GpuRaySensorImpl::GpuRaySensorImpl(sensors::GpuRaySensorPtr gz_RaySensor) : SensorImpl(gz_RaySensor)
	{

	}

	void GpuRaySensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_raysensor()->ConnectUpdated(boost::bind(&GpuRaySensorImpl::OnUpdate,c));

		rrvar_scan_stream->SetMaxBacklog(3);
	}

	laserscan::LaserScanPtr GpuRaySensorImpl::capture_scan()
	{
		laserscan::LaserScanPtr o(new laserscan::LaserScan());
		laserscan::LaserScanInfoPtr info(new laserscan::LaserScanInfo());
		o->scan_info = info;
		sensors::GpuRaySensorPtr c = get_raysensor();

		bool a = c->IsActive();
		c->SetActive(false);

		info->angle_max = c->AngleMax().Radian();
		info->angle_min = c->AngleMin().Radian();
		info->angle_increment = c->AngleResolution();
		info->angle_count = c->RayCount();
		info->vertical_angle_max = c->VerticalAngleMax().Radian();
		info->vertical_angle_min = c->VerticalAngleMin().Radian();
		info->vertical_angle_increment = c->VerticalAngleResolution();
		info->vertical_angle_count = c->VerticalRayCount();
		info->range_min = c->RangeMin();
		info->range_max = c->RangeMax();
		info->range_resolution = c->RangeResolution();
		info->scan_time = 0;
		info->time_increment = 0;

		size_t sample_count = info->angle_count * info->vertical_angle_count;
		o->intensities = RR::AllocateRRArray<double>(sample_count);
		o->ranges = RR::AllocateRRArray<double>(sample_count);
		o->fiducial = RR::AllocateRRArray<int32_t>(sample_count);
		o->color = RR::AllocateEmptyRRNamedArray<com::robotraconteur::image::PixelRGB>(0);

		auto i = &o->intensities->at(0);
		auto r = &o->ranges->at(0);
		auto f = &o->fiducial->at(0);
		for (uint32_t j = 0; j < sample_count; j++)
		{
			i[j] = c->Retro(j);
			r[j] = c->Range(j);
			f[j] = c->Fiducial(j);
		}

		c->SetActive(a);

		return o;
	}

	sensors::GpuRaySensorPtr GpuRaySensorImpl::get_raysensor()
	{
		return std::dynamic_pointer_cast<sensors::GpuRaySensor>(get_sensor());
	}

	void GpuRaySensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<GpuRaySensorImpl> c1=RR_DYNAMIC_POINTER_CAST<GpuRaySensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	void GpuRaySensorImpl::OnUpdate1()
	{		
		auto i = capture_scan();
		rrvar_scan_stream->AsyncSendPacket(i, []() {});		
	}
}
