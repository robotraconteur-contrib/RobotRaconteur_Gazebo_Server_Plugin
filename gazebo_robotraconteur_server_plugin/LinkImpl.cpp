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

#include "LinkImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
  LinkImpl::LinkImpl(physics::LinkPtr l) : EntityImpl(l)
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

	void LinkImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		EntityImpl::OnUpdate1(_info);		
		auto l=gz_link.lock();
		if (!l) return;		
		
		RR::RRListPtr<RR::RRNamedArray<geometry::Wrench> > ft;
		RR::TimeSpec ts;
		uint32_t ep;
		if (rrvar_applied_wrenches->TryGetInValue(ft, ts, ep))
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
		
	RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char> > LinkImpl::get_sensor_names()
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

	void LinkImpl::attach_link(const std::string& model2, const std::string& link2)
	{
		// Based on https://github.com/pal-robotics/gazebo_ros_link_attacher

		physics::LinkPtr l1 = get_link();

		LinkImpl_attached_joint j;
		if(this->get_attached_link_joint(model2, link2, j)){			
			j.joint->Attach(l1, j.l2);
			return;
		}
		
		j.model2 = model2;
		j.link2 = link2;
		
		physics::BasePtr b1 = l1->GetModel();
	
		physics::BasePtr b2 = l1->GetWorld()->ModelByName(model2);
		if (b2 == NULL){
			throw RR::InvalidArgumentException("Invalid model name for link attach: " + model2);
		}
		
		physics::ModelPtr m1(dynamic_cast<physics::Model*>(b1.get()));
		
		physics::ModelPtr m2(dynamic_cast<physics::Model*>(b2.get()));
		j.m2 = m2;
		
		physics::LinkPtr l2 = m2->GetLink(link2);
		if (l2 == NULL){
			throw RR::InvalidArgumentException("Invalid link name for link attach: " + link2);
		
		}
		
		j.l2 = l2;

		
		j.joint = l1->GetWorld()->Physics()->CreateJoint("fixed", m1);
		this->attached_link_joints.push_back(j);

		
		j.joint->Attach(l1, l2);
		
		j.joint->Load(l1, l2, ignition::math::Pose3d());
		
		j.joint->SetModel(m1);
		/*
		* If SetModel is not done we get:
		* ***** Internal Program Error - assertion (this->GetParentModel() != __null)
		failed in void gazebo::physics::Entity::PublishPose():
		/tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
		An entity without a parent model should not happen
		* If SetModel is given the same model than CreateJoint given
		* Gazebo crashes with
		* ***** Internal Program Error - assertion (self->inertial != __null)
		failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
		/tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
		*/

		
		/*j.joint->SetUpperLimit(0, 0);
		ROS_DEBUG_STREAM("SetLowStop");
		j.joint->SetLowerLimit(0, 0);
		ROS_DEBUG_STREAM("Init");*/
		j.joint->Init();
		
	}

	void LinkImpl::attach_link_with_pose(const std::string& model, const std::string& link_name, const com::robotraconteur::geometry::Pose& pose)
	{
		// Based on https://github.com/pal-robotics/gazebo_ros_link_attacher
		throw RR::NotImplementedException("Not implemented");
	}

	void LinkImpl::detach_link(const std::string& model2, const std::string& link2)
	{
		// Based on https://github.com/pal-robotics/gazebo_ros_link_attacher

		LinkImpl_attached_joint j;
		if(this->get_attached_link_joint(model2, link2, j)){
			j.joint->Detach();
			return;
		}

		throw RR::InvalidArgumentException("Model " + model2 + " link " + link2 + " is not attached");
	}

	bool LinkImpl::get_attached_link_joint(std::string model2, std::string link2, LinkImpl_attached_joint &joint)
	{
		LinkImpl_attached_joint j;
		for(std::vector<LinkImpl_attached_joint>::iterator it = this->attached_link_joints.begin(); it != this->attached_link_joints.end(); ++it){
			j = *it;
			if ((j.model2.compare(model2) == 0)
					&& (j.link2.compare(link2) == 0)){
				joint = j;
				return true;
			}
		}
		return false;
	}
}
