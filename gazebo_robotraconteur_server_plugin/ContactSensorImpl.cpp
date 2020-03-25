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

	void ContactSensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_contactsensor()->ConnectUpdated(boost::bind(&ContactSensorImpl::OnUpdate,c));

		boost::weak_ptr<ContactSensorImpl> weak_this = RR::rr_cast<ContactSensorImpl>(shared_from_this());
		this->rrvar_contacts->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Sensor has been released");
				return this_->CaptureContacts();
			}
		);
	}

	void ContactSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<ContactSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<ContactSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	RR::RRListPtr<rrgz::Contact> ContactSensorImpl::CaptureContacts()
	{
		auto o=RR::AllocateEmptyRRList<rrgz::Contact>();
		sensors::ContactSensorPtr c=get_contactsensor();


		msgs::Contacts contacts=c->Contacts();
		int count=contacts.contact_size();
		for (int i=0; i<count; i++)
		{
			msgs::Contact contact=contacts.contact(i);
			rrgz::ContactPtr o_i(new rrgz::Contact());
			o_i->contact_name1=contact.collision1();
			o_i->contact_name2=contact.collision2();
			o->push_back(o_i);
		}

		return o;
}

	sensors::ContactSensorPtr ContactSensorImpl::get_contactsensor()
	{
		return std::dynamic_pointer_cast<sensors::ContactSensor>(get_sensor());
	}

	void ContactSensorImpl::OnUpdate1()
	{
		auto i=CaptureContacts();
		rrvar_contacts->SetOutValue(i);
	}

}
