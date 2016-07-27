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

	double SonarSensorImpl::get_RangeMin()
	{
		return get_sonarsensor()->RangeMin();
	}
	void SonarSensorImpl::set_RangeMin(double value)
	{
		throw std::runtime_error("Read only property");
	}

	double SonarSensorImpl::get_RangeMax()
	{
		return get_sonarsensor()->RangeMax();
	}
	void SonarSensorImpl::set_RangeMax(double value)
	{
		throw std::runtime_error("Read only property");
	}

	double SonarSensorImpl::get_Radius()
	{
		return get_sonarsensor()->Radius();
	}
	void SonarSensorImpl::set_Radius(double value)
	{
		throw std::runtime_error("Read only property");
	}


	double SonarSensorImpl::get_Range()
	{
		boost::mutex::scoped_lock lock(this_lock);
		sensors::SonarSensorPtr c=get_sonarsensor();
		return c->Range();
	}
	void SonarSensorImpl::set_Range(double value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::Wire<double> > SonarSensorImpl::get_RangeWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_RangeWire;
	}
	void SonarSensorImpl::set_RangeWire(RR_SHARED_PTR<RR::Wire<double> > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		m_RangeWire=value;
		m_RangeWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<double> >();
		m_RangeWire_b->Init(m_RangeWire);
	}

	sensors::SonarSensorPtr SonarSensorImpl::get_sonarsensor()
	{
		return std::dynamic_pointer_cast<sensors::SonarSensor>(get_sensor());
	}

	void SonarSensorImpl::OnUpdate1()
	{
		RR_SHARED_PTR<RR::WireBroadcaster<double> > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_RangeWire_b;
		}
		if (b)
		{
			auto i=get_Range();
			b->SetOutValue(i);
		}
	}

}
