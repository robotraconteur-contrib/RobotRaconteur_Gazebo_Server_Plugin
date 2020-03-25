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
}
