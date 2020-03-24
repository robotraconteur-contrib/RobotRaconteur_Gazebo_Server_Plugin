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

	void LinkImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		EntityImpl::OnUpdate1(_info);		
		physics::LinkPtr l=get_link();
		
		RR::WireUnicastReceiverPtr<RR::RRListPtr<RR::RRNamedArray<geometry::Wrench> > > appliedft_u;
		{
			boost::mutex::scoped_lock lock(Link_default_abstract_impl::this_lock);
			appliedft_u = rrvar_AppliedWrenches;
		}

		if (appliedft_u)
		{
			RR::RRListPtr<RR::RRNamedArray<geometry::Wrench> > ft;
			RR::TimeSpec ts;
			uint32_t ep;
			if (appliedft_u->TryGetInValue(ft, ts, ep))
			{
				if (ft)
				{
					for (auto e : *ft)
					{
						if (!e) continue;
						auto e2 = RR::RRNamedArrayToScalar(e);
						
						if (!(e)->size() != 1) continue;
						ignition::math::Vector3d torque(e2.s.torque.s.x, e2.s.torque.s.y, e2.s.torque.s.z);
						ignition::math::Vector3d force(e2.s.force.s.x, e2.s.force.s.y, e2.s.force.s.z);
						l->AddRelativeForce(force);
						l->AddRelativeTorque(torque);
					}
				}
			}
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
