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

#include "EntityImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
std::string EntityImpl::get_Name()
	{
		return get_entity()->GetName();
	}
	void EntityImpl::set_Name(std::string value)
	{
		throw std::runtime_error("Read only property");
	}
	std::string EntityImpl::get_ScopedName()
	{
		return get_entity()->GetScopedName(true);
	}
	void EntityImpl::set_ScopedName(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	void EntityImpl::Init(const std::string& rr_path)
	{
		RR_WEAK_PTR<EntityImpl> w1=shared_from_this();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&EntityImpl::OnUpdate, w1, _1));

		gz_path=get_entity()->GetScopedName(true);
		rr_context=RR::ServerContext::GetCurrentServerContext();
		this->rr_path=rr_path;

		this->deleteEntityConnection = event::Events::ConnectDeleteEntity(
				boost::bind(&EntityImpl::OnEntityDeleted, w1, _1));
	}

	void EntityImpl::OnUpdate(RR_WEAK_PTR<EntityImpl> e, const common::UpdateInfo & _info)
	{
		RR_SHARED_PTR<EntityImpl> e1=e.lock();
		if (!e1) return;
		e1->OnUpdate1(_info);
	}

	void EntityImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::Pose > > > wpose;
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::Pose > > > rpose;
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > wvel;
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > rvel;
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > wacc;
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > > racc;
		{
			boost::mutex::scoped_lock lock(this_lock);
			wpose=m_WorldPoseGetWire_b;
			rpose=m_RelativePoseGetWire_b;
			wvel=m_WorldVelocityGetWire_b;
			rvel=m_RelativeVelocityGetWire_b;
			wacc=m_WorldAccelerationGetWire_b;
			racc=m_RelativeAccelerationGetWire_b;

		}

		RR_SHARED_PTR<rrgz::Pose > wpose1=get_WorldPose();
		RR_SHARED_PTR<rrgz::Pose > rpose1=get_RelativePose();
		RR_SHARED_PTR<RR::RRArray<double > > wvel1=get_WorldVelocity();
		RR_SHARED_PTR<RR::RRArray<double > > rvel1=get_RelativeVelocity();
		RR_SHARED_PTR<RR::RRArray<double > > wacc1=get_WorldAcceleration();
		RR_SHARED_PTR<RR::RRArray<double > > racc1=get_RelativeAcceleration();

		if (wpose) wpose->SetOutValue(wpose1);
		if (rpose) rpose->SetOutValue(rpose1);
		if (wvel) wvel->SetOutValue(wvel1);
		if (rvel) rvel->SetOutValue(rvel1);
		if (wacc) wacc->SetOutValue(wacc1);
		if (racc) racc->SetOutValue(racc1);
	}

	void EntityImpl::OnEntityDeleted(RR_WEAK_PTR<EntityImpl> t, const std::string& path)
	{

		//TODO: Attempt to support entity deletion. This doesn't seem to work yet.
		RR_SHARED_PTR<EntityImpl> t1=t.lock();
		if(!t1) return;
		RR_SHARED_PTR<RR::ServerContext> c=t1->rr_context.lock();
		if(!c) return;

		if (path == t1->gz_path)
		{
			c->ReleaseServicePath(t1->rr_path);
		}
	}

	std::string EntityImpl::GetRRPath()
	{
		return rr_path;
	}

	RR_SHARED_PTR<rrgz::Pose > EntityImpl::get_WorldPose()
	{
		math::Pose p=get_entity()->GetWorldPose();
		RR_SHARED_PTR<rrgz::Pose > o=RR_MAKE_SHARED<rrgz::Pose>();
		o->Position=RR::AllocateRRArray<double>(3);
		o->Orientation=RR::AllocateRRArray<double>(4);

		for (uint32_t i=0; i<3; i++) (*o->Position)[i]=p.pos[i];
		(*o->Orientation)[0]=p.rot.w;
		(*o->Orientation)[1]=p.rot.x;
		(*o->Orientation)[2]=p.rot.y;
		(*o->Orientation)[3]=p.rot.z;
		return o;
	}
	void EntityImpl::set_WorldPose(RR_SHARED_PTR<rrgz::Pose > value)
	{
		RR_NULL_CHECK(value);
		RR_NULL_CHECK(value->Position);
		RR_NULL_CHECK(value->Orientation);

		if (value->Position->size()!=3) throw std::invalid_argument("Invalid vector");
		if (value->Orientation->size()!=4) throw std::invalid_argument("Invalid vector");

		double* v1=value->Position->ptr();
		double* q1=value->Orientation->ptr();

		math::Vector3 v(v1[0], v1[1], v1[2]);
		math::Quaternion q(q1[0], q1[1], q1[2], q1[3]);

		math::Pose p(v,q);
		get_entity()->SetWorldPose(p, true, true);
	}

	RR_SHARED_PTR<rrgz::Pose > EntityImpl::get_RelativePose()
	{
		math::Pose p=get_entity()->GetRelativePose();
		RR_SHARED_PTR<rrgz::Pose > o=RR_MAKE_SHARED<rrgz::Pose>();
		o->Position=RR::AllocateRRArray<double>(3);
		o->Orientation=RR::AllocateRRArray<double>(4);

		for (uint32_t i=0; i<3; i++) (*o->Position)[i]=p.pos[i];
		(*o->Orientation)[0]=p.rot.w;
		(*o->Orientation)[1]=p.rot.x;
		(*o->Orientation)[2]=p.rot.y;
		(*o->Orientation)[3]=p.rot.z;
		return o;
	}
	void EntityImpl::set_RelativePose(RR_SHARED_PTR<rrgz::Pose > value)
	{
		RR_NULL_CHECK(value);
		RR_NULL_CHECK(value->Position);
		RR_NULL_CHECK(value->Orientation);

		if (value->Position->size()!=3) throw std::invalid_argument("Invalid vector");
		if (value->Orientation->size()!=4) throw std::invalid_argument("Invalid vector");


		double* v1=value->Position->ptr();
		double* q1=value->Orientation->ptr();

		math::Vector3 v(v1[0], v1[1], v1[2]);
		math::Quaternion q(q1[0], q1[1], q1[2], q1[3]);

		math::Pose p(v,q);
		get_entity()->SetRelativePose(p, true, true);
	}

	RR_SHARED_PTR<RR::RRArray<double > > EntityImpl::get_WorldVelocity()
	{
		RR_SHARED_PTR<RR::RRArray<double> > o=RR::AllocateRRArray<double>(6);

		math::Vector3 a=get_entity()->GetWorldAngularVel();
		math::Vector3 v=get_entity()->GetWorldLinearVel();
		(*o)[0]=a.x; (*o)[1]=a.y; (*o)[2]=a.z;
		(*o)[3]=v.x; (*o)[4]=v.y; (*o)[5]=v.z;
		return o;
	}
	void EntityImpl::set_WorldVelocity(RR_SHARED_PTR<RR::RRArray<double > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::RRArray<double > > EntityImpl::get_RelativeVelocity()
	{
		RR_SHARED_PTR<RR::RRArray<double> > o=RR::AllocateRRArray<double>(6);

		math::Vector3 a=get_entity()->GetRelativeAngularVel();
		math::Vector3 v=get_entity()->GetRelativeLinearVel();
		(*o)[0]=a.x; (*o)[1]=a.y; (*o)[2]=a.z;
		(*o)[3]=v.x; (*o)[4]=v.y; (*o)[5]=v.z;
		return o;
	}
	void EntityImpl::set_RelativeVelocity(RR_SHARED_PTR<RR::RRArray<double > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::RRArray<double > > EntityImpl::get_WorldAcceleration()
	{
		RR_SHARED_PTR<RR::RRArray<double> > o=RR::AllocateRRArray<double>(6);

			math::Vector3 a=get_entity()->GetWorldAngularAccel();
			math::Vector3 v=get_entity()->GetWorldLinearAccel();
			(*o)[0]=a.x; (*o)[1]=a.y; (*o)[2]=a.z;
			(*o)[3]=v.x; (*o)[4]=v.y; (*o)[5]=v.z;
			return o;
	}
	void EntityImpl::set_WorldAcceleration(RR_SHARED_PTR<RR::RRArray<double > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::RRArray<double > > EntityImpl::get_RelativeAcceleration()
	{
		RR_SHARED_PTR<RR::RRArray<double> > o=RR::AllocateRRArray<double>(6);

			math::Vector3 a=get_entity()->GetRelativeAngularAccel();
			math::Vector3 v=get_entity()->GetRelativeLinearAccel();
			(*o)[0]=a.x; (*o)[1]=a.y; (*o)[2]=a.z;
			(*o)[3]=v.x; (*o)[4]=v.y; (*o)[5]=v.z;
			return o;
	}
	void EntityImpl::set_RelativeAcceleration(RR_SHARED_PTR<RR::RRArray<double > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > EntityImpl::get_WorldPoseGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_WorldPoseGetWire;
	}
	void EntityImpl::set_WorldPoseGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_WorldPoseGetWire) throw std::runtime_error("Already set");
		m_WorldPoseGetWire=value;
		m_WorldPoseGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::Pose> > >();
		m_WorldPoseGetWire_b->Init(m_WorldPoseGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > EntityImpl::get_RelativePoseGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_RelativePoseGetWire;
	}
	void EntityImpl::set_RelativePoseGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<rrgz::Pose > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_RelativePoseGetWire) throw std::runtime_error("Already set");
		m_RelativePoseGetWire=value;
		m_RelativePoseGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<rrgz::Pose> > >();
		m_RelativePoseGetWire_b->Init(m_RelativePoseGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > EntityImpl::get_WorldVelocityGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_WorldVelocityGetWire;
	}
	void EntityImpl::set_WorldVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_WorldVelocityGetWire) throw std::runtime_error("Already set");
		m_WorldVelocityGetWire=value;
		m_WorldVelocityGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > >();
		m_WorldVelocityGetWire_b->Init(m_WorldVelocityGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > EntityImpl::get_RelativeVelocityGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_RelativeVelocityGetWire;
	}
	void EntityImpl::set_RelativeVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_RelativeVelocityGetWire) throw std::runtime_error("Already set");
		m_RelativeVelocityGetWire=value;
		m_RelativeVelocityGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > >();
		m_RelativeVelocityGetWire_b->Init(m_RelativeVelocityGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > EntityImpl::get_WorldAccelerationGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_WorldAccelerationGetWire;
	}
	void EntityImpl::set_WorldAccelerationGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_WorldAccelerationGetWire) throw std::runtime_error("Already set");
		m_WorldAccelerationGetWire=value;
		m_WorldAccelerationGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > >();
		m_WorldAccelerationGetWire_b->Init(m_WorldAccelerationGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > EntityImpl::get_RelativeAccelerationGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_RelativeAccelerationGetWire;
	}
	void EntityImpl::set_RelativeAccelerationGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRArray<double > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_RelativeAccelerationGetWire) throw std::runtime_error("Already set");
		m_RelativeAccelerationGetWire=value;
		m_RelativeAccelerationGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRArray<double > > > >();
		m_RelativeAccelerationGetWire_b->Init(m_RelativeAccelerationGetWire);
	}

}
