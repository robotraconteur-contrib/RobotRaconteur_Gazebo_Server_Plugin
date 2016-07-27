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
	ContactSensorImpl::ContactSensorImpl(sensors::ContactSensorPtr gz_contact) : SensorImpl(gz_contact)
	{

	}

	void ContactSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_contactsensor()->ConnectUpdated(boost::bind(&ContactSensorImpl::OnUpdate,c));
	}

	void ContactSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<ContactSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<ContactSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > ContactSensorImpl::CaptureContacts()
	{
		boost::mutex::scoped_lock lock(this_lock);
		auto o=RR_MAKE_SHARED<RR::RRList<rrgz::Contact  > >();
		sensors::ContactSensorPtr c=get_contactsensor();


		msgs::Contacts contacts=c->Contacts();
		int count=contacts.contact_size();
		for (int i=0; i<count; i++)
		{
			msgs::Contact contact=contacts.contact(i);
			auto o_i=RR_MAKE_SHARED<rrgz::Contact>();
			o_i->contactName1=contact.collision1();
			o_i->contactName2=contact.collision2();
			o->list.push_back(o_i);
		}

		return o;
}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > > ContactSensorImpl::get_ContactWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_ContactWire;
	}
	void ContactSensorImpl::set_ContactWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		m_ContactWire=value;
		m_ContactWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > >();
		m_ContactWire_b->Init(m_ContactWire);
	}

	sensors::ContactSensorPtr ContactSensorImpl::get_contactsensor()
	{
		return std::dynamic_pointer_cast<sensors::ContactSensor>(get_sensor());
	}

	void ContactSensorImpl::OnUpdate1()
	{
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRList<rrgz::Contact  > > > > b;
		{
		boost::mutex::scoped_lock lock(this_lock);
		b=m_ContactWire_b;
		}
		if (b)
		{
			auto i=CaptureContacts();
			b->SetOutValue(i);
		}
	}

}
