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

#include "SensorImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	SensorImpl::SensorImpl(sensors::SensorPtr gz_sensor)
	{
		this->gz_sensor=gz_sensor;
	}

	std::string SensorImpl::get_Name()
	{
		return get_sensor()->ScopedName();
	}
	void SensorImpl::set_Name(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	std::string SensorImpl::get_Type()
	{
		return get_sensor()->Type();
	}

	void SensorImpl::set_Type(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	std::string SensorImpl::get_ParentName()
	{
		return get_sensor()->ParentName();
	}
	void SensorImpl::set_ParentName(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<rrgz::Pose > SensorImpl::get_Pose()
	{
		math::Pose p=get_sensor()->Pose();
		RR_SHARED_PTR<rrgz::Pose > o=RR_MAKE_SHARED<rrgz::Pose>();
		o->Position=RR::AllocateRRArray<double>(3);
		o->Orientation=RR::AllocateRRArray<double>(4);

		for (uint32_t i=0; i<3; i++) (*o->Position)[i]=p.pos[i];
		(*o->Orientation)[0]=p.rot.w;
		(*o->Orientation)[1]=p.rot.x;
		(*o->Orientation)[2]=p.rot.y;
		(*o->Orientation)[3]=p.rot.z;
		return o;
	}
	void SensorImpl::set_Pose(RR_SHARED_PTR<rrgz::Pose > value)
	{
		throw std::runtime_error("Read only property");
	}

	sensors::SensorPtr SensorImpl::get_sensor()
	{
		sensors::SensorPtr s=gz_sensor.lock();
		if (!s) throw std::runtime_error("Sensor has been released");
		return s;
	}

	uint8_t SensorImpl::get_Active()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return get_sensor()->IsActive() ? 1 : 0;
	}
	void SensorImpl::set_Active(uint8_t value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		get_sensor()->SetActive(value!=0);
	}

	double SensorImpl::get_UpdateRate()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return get_sensor()->UpdateRate();
	}
	void SensorImpl::set_UpdateRate(double value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		get_sensor()->SetUpdateRate(value);
	}

	double SensorImpl::get_LastUpdateTime()
	{
		return get_sensor()->LastUpdateTime().Double();
	}
	void SensorImpl::set_LastUpdateTime(double value)
	{
		throw std::runtime_error("Read only property");
	}

	double SensorImpl::get_LastMeasurementTime()
	{
		return get_sensor()->LastMeasurementTime().Double();
	}
	void SensorImpl::set_LastMeasurementTime(double value)
	{
		throw std::runtime_error("Read only property");
	}
}
