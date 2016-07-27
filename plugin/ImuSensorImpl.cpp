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
	ImuSensorImpl::ImuSensorImpl(sensors::ImuSensorPtr gz_imu) : SensorImpl(gz_imu)
	{

	}

	void ImuSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_imusensor()->ConnectUpdated(boost::bind(&ImuSensorImpl::OnUpdate,c));
	}

	void ImuSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<ImuSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<ImuSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}


	RR_SHARED_PTR<rrgz::ImuState > ImuSensorImpl::get_ReadState()
	{
		boost::mutex::scoped_lock lock(this_lock);
		sensors::ImuSensorPtr c=get_imusensor();
		auto o=RR_MAKE_SHARED<rrgz::ImuState>();
		auto a_v=RR::AllocateRRArray<double>(3);
		auto l_a=RR::AllocateRRArray<double>(3);
		auto o_p=RR::AllocateRRArray<double>(4);

		auto a_v1=c->AngularVelocity();
		auto l_a1=c->LinearAcceleration();
		auto o_p1=c->Orientation();

		(*a_v)[0]=a_v1[0];
		(*a_v)[1]=a_v1[1];
		(*a_v)[2]=a_v1[2];

		(*l_a)[0]=l_a1[0];
		(*l_a)[1]=l_a1[1];
		(*l_a)[2]=l_a1[2];

		(*o_p)[0]=o_p1.W();
		(*o_p)[1]=o_p1.X();
		(*o_p)[2]=o_p1.Y();
		(*o_p)[3]=o_p1.Z();

		return o;
	}
	void ImuSensorImpl::set_ReadState(RR_SHARED_PTR<rrgz::ImuState> value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::ImuState > > > ImuSensorImpl::get_StateWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_StateWire;
	}
	void ImuSensorImpl::set_StateWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::ImuState > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		m_StateWire=value;
		m_StateWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::ImuState> > >();
		m_StateWire_b->Init(m_StateWire);
	}

	void ImuSensorImpl::SetReferencePose()
	{
		get_imusensor()->SetReferencePose();
	}

	sensors::ImuSensorPtr ImuSensorImpl::get_imusensor()
	{
		return std::dynamic_pointer_cast<sensors::ImuSensor>(get_sensor());
	}

	void ImuSensorImpl::OnUpdate1()
	{
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::ImuState > > > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_StateWire_b;
		}
		if (b)
		{
			auto i=get_ReadState();
			b->SetOutValue(i);
		}
	}

}
