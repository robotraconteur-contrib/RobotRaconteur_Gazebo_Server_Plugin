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

#include "LinkImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
  LinkImpl::LinkImpl(physics::LinkPtr l)
	{
		gz_link=l;
		model_name=l->GetParentModel()->GetName();
		gz_world=l->GetParentModel()->GetWorld();
	}

	physics::LinkPtr LinkImpl::get_link()
	{
		physics::LinkPtr l=gz_link.lock();
		if (!l) throw std::runtime_error("Link has been released");
		return l;
	}

	physics::EntityPtr LinkImpl::get_entity()
	{
		return boost::dynamic_pointer_cast<physics::Entity>(get_link());
	}

	RR::RRListPtr<RR::RRArray<double > > LinkImpl::get_AppliedWrenches()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return applied_wrenches;
	}
	void LinkImpl::set_AppliedWrenches(RR::RRListPtr<RR::RRArray<double > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (!value)
		{
			applied_wrenches=value;
			return;
		}
		for (auto e=value->begin(); e!=value->end(); e++)
		{
			RR_NULL_CHECK(*e);
			if ((*e)->size()!=6) throw std::invalid_argument("Invalid vector length");
		}

		applied_wrenches=value;
	}
	void LinkImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		EntityImpl::OnUpdate1(_info);
		boost::mutex::scoped_lock lock(this_lock);
		physics::LinkPtr l=get_link();
		if (applied_wrenches)
		{
			for (auto e=applied_wrenches->begin(); e!=applied_wrenches->end(); e++)
			{
				ignition::math::Vector3d torque((**e)[0], (**e)[1], (**e)[2]);
				ignition::math::Vector3d force((**e)[3], (**e)[4], (**e)[5]);
				l->AddRelativeForce(force);
				l->AddRelativeTorque(torque);
			}
		}

		try
		{
			if (m_AppliedWrenchesSetWire_conn)
			{
				if (m_AppliedWrenchesSetWire_conn->GetInValueValid())
				{
					auto inval=m_AppliedWrenchesSetWire_conn->GetInValue();
					if (inval)
					{
						for (auto e=inval->begin(); e!=inval->end(); e++)
						{
							if (!*e) continue;
							if (!(*e)->size()!=6) continue;
							ignition::math::Vector3d torque((**e)[0], (**e)[1], (**e)[2]);
							ignition::math::Vector3d force((**e)[3], (**e)[4], (**e)[5]);
							l->AddRelativeForce(force);
							l->AddRelativeTorque(torque);
						}
					}
				}
			}
		}
		catch (std::exception&) {}
	}

	RR::WirePtr<RR::RRListPtr<RR::RRArray<double> > > LinkImpl::get_AppliedWrenchesSetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_AppliedWrenchesSetWire;
	}
	void LinkImpl::set_AppliedWrenchesSetWire(RR::WirePtr<RR::RRListPtr<RR::RRArray<double > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_AppliedWrenchesSetWire) throw std::runtime_error("Read only property");
		m_AppliedWrenchesSetWire=value;
		RR_WEAK_PTR<LinkImpl> l=boost::dynamic_pointer_cast<LinkImpl>(shared_from_this());
		m_AppliedWrenchesSetWire->SetWireConnectCallback(boost::bind(&LinkImpl::OnAppliedWrenchesSetWireConnect,l,_1));
	}

	void LinkImpl::OnAppliedWrenchesSetWireConnect(RR_WEAK_PTR<LinkImpl> l, RR::WireConnectionPtr<RR::RRListPtr<RR::RRArray<double> > > connection)
	{
		RR_SHARED_PTR<LinkImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		//if(l1->m_AppliedWrenchesSetWire_conn) throw std::runtime_error("Wire in use");
		l1->m_AppliedWrenchesSetWire_conn=connection;
		l1->m_AppliedWrenchesSetWire_conn->SetWireConnectionClosedCallback(boost::bind(&LinkImpl::OnAppliedWrenchesSetWireDisconnect,l,_1));
	}

	void LinkImpl::OnAppliedWrenchesSetWireDisconnect(RR_WEAK_PTR<LinkImpl> l, RR::WireConnectionPtr<RR::RRListPtr<RR::RRArray<double> > > connection)
	{
		RR_SHARED_PTR<LinkImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		if(l1->m_AppliedWrenchesSetWire_conn==connection)
		{
			l1->m_AppliedWrenchesSetWire_conn.reset();
		}
	}

	RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char> > LinkImpl::get_SensorNames()
	{
		auto o=RR::AllocateEmptyRRList<RR::RRArray<char> >();
		auto l=get_link();
		auto n=l->GetSensorCount();
		for (unsigned int i=0; i<n; i++)
		{
			o->push_back(RR::stringToRRArray(l->GetSensorName(i)));
		}
		return o;
	}	

}
