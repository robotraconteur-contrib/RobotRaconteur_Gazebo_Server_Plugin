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
	SonarSensorImpl::SonarSensorImpl(sensors::SonarSensorPtr gz_sonar) : SensorImpl(gz_sonar)
	{

	}

	void SonarSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_sonarsensor()->ConnectUpdated(boost::bind(&SonarSensorImpl::OnUpdate,c));
	}

	void SonarSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<SonarSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<SonarSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	double SonarSensorImpl::get_range_min()
	{
		return get_sonarsensor()->RangeMin();
	}
	
	double SonarSensorImpl::get_range_max()
	{
		return get_sonarsensor()->RangeMax();
	}
	
	double SonarSensorImpl::get_radius()
	{
		return get_sonarsensor()->Radius();
	}
	
	void SonarSensorImpl::set_range(RR::WirePtr<double> value)
	{
		SonarSensor_default_abstract_impl::set_range(value);
		boost::weak_ptr<SonarSensorImpl> weak_this = RR::rr_cast<SonarSensorImpl>(shared_from_this());
		this->rrvar_range->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				return this_->get_sonarsensor()->Range();
			}
		);
	}

	sensors::SonarSensorPtr SonarSensorImpl::get_sonarsensor()
	{
		return std::dynamic_pointer_cast<sensors::SonarSensor>(get_sensor());
	}

	void SonarSensorImpl::OnUpdate1()
	{
		RR::WireBroadcasterPtr<double> b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=rrvar_range;
		}
		if (b)
		{
			auto i = get_sonarsensor()->Range();
			b->SetOutValue(i);
		}
	}

}
