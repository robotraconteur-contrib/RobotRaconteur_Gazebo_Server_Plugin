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
	ForceTorqueSensorImpl::ForceTorqueSensorImpl(sensors::ForceTorqueSensorPtr gz_contact) : SensorImpl(gz_contact)
	{

	}

	void ForceTorqueSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_forcetorquesensor()->ConnectUpdated(boost::bind(&ForceTorqueSensorImpl::OnUpdate,c));
	}

	void ForceTorqueSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<ForceTorqueSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<ForceTorqueSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}


	RR_SHARED_PTR<RR::RRArray<double> > ForceTorqueSensorImpl::get_ForceTorque()
	{
		boost::mutex::scoped_lock lock(this_lock);
		sensors::ForceTorqueSensorPtr c=get_forcetorquesensor();
		auto t=c->Torque();
		auto f= c->Force();
		auto o=RR::AllocateRRArray<double>(6);
		(*o)[0]=t[0];
		(*o)[1]=t[1];
		(*o)[2]=t[2];
		(*o)[3]=f[0];
		(*o)[4]=f[1];
		(*o)[5]=f[2];
		return o;
	}
	void ForceTorqueSensorImpl::set_ForceTorque(RR_SHARED_PTR<RR::RRArray<double> > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double> > > > ForceTorqueSensorImpl::get_ForceTorqueWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_ForceTorqueWire;
	}
	void ForceTorqueSensorImpl::set_ForceTorqueWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double> > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		m_ForceTorqueWire=value;
		m_ForceTorqueWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double> > > >();
		m_ForceTorqueWire_b->Init(m_ForceTorqueWire);
	}

	sensors::ForceTorqueSensorPtr ForceTorqueSensorImpl::get_forcetorquesensor()
	{
		return std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(get_sensor());
	}

	void ForceTorqueSensorImpl::OnUpdate1()
	{
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double> > > > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_ForceTorqueWire_b;
		}
		if (b)
		{
			auto i=get_ForceTorque();
			b->SetOutValue(i);
		}
	}

}
