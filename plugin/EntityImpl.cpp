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

namespace math=ignition::math;

namespace RobotRaconteurGazeboServerPlugin
{
std::string EntityImpl::get_Name()
	{
		return get_entity()->GetName();
	}
	
	std::string EntityImpl::get_ScopedName()
	{
		return get_entity()->GetScopedName(true);
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
		RR_SHARED_PTR<RR::WireBroadcaster<rrgz::PosePtr > > wpose;
		RR_SHARED_PTR<RR::WireBroadcaster<rrgz::PosePtr > > rpose;
		RR_SHARED_PTR<RR::WireBroadcaster<RR::RRArrayPtr<double > > > wvel;
		RR_SHARED_PTR<RR::WireBroadcaster<RR::RRArrayPtr<double > > > rvel;
		RR_SHARED_PTR<RR::WireBroadcaster<RR::RRArrayPtr<double > > > wacc;
		RR_SHARED_PTR<RR::WireBroadcaster<RR::RRArrayPtr<double > > > racc;
		{
			boost::mutex::scoped_lock lock(this_lock);
			wpose=m_WorldPoseGetWire_b;
			rpose=m_RelativePoseGetWire_b;
			wvel=m_WorldVelocityGetWire_b;
			rvel=m_RelativeVelocityGetWire_b;
			wacc=m_WorldAccelerationGetWire_b;
			racc=m_RelativeAccelerationGetWire_b;

		}

		auto wpose1=get_WorldPose();
		auto rpose1=get_RelativePose();
		auto wvel1=get_WorldVelocity();
		auto rvel1=get_RelativeVelocity();
		auto wacc1=get_WorldAcceleration();
		auto racc1=get_RelativeAcceleration();

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

	rrgz::PosePtr EntityImpl::get_WorldPose()
	{
		auto p=get_entity()->WorldPose();
		rrgz::PosePtr o(new rrgz::Pose());
		o->Position=RR::AllocateRRArray<double>(3);
		o->Orientation=RR::AllocateRRArray<double>(4);

		for (uint32_t i=0; i<3; i++) (*o->Position)[i]=p.Pos()[i];
		(*o->Orientation)[0]=p.Rot().W();
		(*o->Orientation)[1]=p.Rot().X();
		(*o->Orientation)[2]=p.Rot().Y();
		(*o->Orientation)[3]=p.Rot().Z();
		return o;
	}
	void EntityImpl::set_WorldPose(rrgz::PosePtr value)
	{
		RR_NULL_CHECK(value);
		RR_NULL_CHECK(value->Position);
		RR_NULL_CHECK(value->Orientation);

		if (value->Position->size()!=3) throw std::invalid_argument("Invalid vector");
		if (value->Orientation->size()!=4) throw std::invalid_argument("Invalid vector");

		double* v1=&value->Position->at(0);
		double* q1=&value->Orientation->at(0);

		math::Vector3d v(v1[0], v1[1], v1[2]);
		math::Quaterniond q(q1[0], q1[1], q1[2], q1[3]);

		math::Pose3d p(v,q);
		get_entity()->SetWorldPose(p, true, true);
	}

	rrgz::PosePtr EntityImpl::get_RelativePose()
	{
		auto p=get_entity()->RelativePose();
		rrgz::PosePtr o=(new rrgz::Pose());
		o->Position=RR::AllocateRRArray<double>(3);
		o->Orientation=RR::AllocateRRArray<double>(4);

		for (uint32_t i=0; i<3; i++) (*o->Position)[i]=p.Pos()[i];
		(*o->Orientation)[0]=p.Rot().W();
		(*o->Orientation)[1]=p.Rot().X();
		(*o->Orientation)[2]=p.Rot().Y();
		(*o->Orientation)[3]=p.Rot().Z();
		return o;
	}
	void EntityImpl::set_RelativePose(rrgz::PosePtr value)
	{
		RR_NULL_CHECK(value);
		RR_NULL_CHECK(value->Position);
		RR_NULL_CHECK(value->Orientation);

		if (value->Position->size()!=3) throw std::invalid_argument("Invalid vector");
		if (value->Orientation->size()!=4) throw std::invalid_argument("Invalid vector");


		double* v1=&value->Position->at(0);
		double* q1=&value->Orientation->at(0);

		math::Vector3d v(v1[0], v1[1], v1[2]);
		math::Quaterniond q(q1[0], q1[1], q1[2], q1[3]);

		math::Pose3d p(v,q);
		get_entity()->SetRelativePose(p, true, true);
	}

	RR::RRArrayPtr<double> EntityImpl::get_WorldVelocity()
	{
		auto o=RR::AllocateRRArray<double>(6);

		math::Vector3d a=get_entity()->WorldAngularVel();
		math::Vector3d v=get_entity()->WorldLinearVel();
		(*o)[0]=a.X(); (*o)[1]=a.Y(); (*o)[2]=a.Z();
		(*o)[3]=v.X(); (*o)[4]=v.Y(); (*o)[5]=v.Z();
		return o;
	}
	
	RR::RRArrayPtr<double > EntityImpl::get_RelativeVelocity()
	{
		auto o=RR::AllocateRRArray<double>(6);

		math::Vector3d a=get_entity()->RelativeAngularVel();
		math::Vector3d v=get_entity()->RelativeLinearVel();
		(*o)[0]=a.X(); (*o)[1]=a.Y(); (*o)[2]=a.Z();
		(*o)[3]=v.X(); (*o)[4]=v.Y(); (*o)[5]=v.Z();
		return o;
	}
	
	RR::RRArrayPtr<double > EntityImpl::get_WorldAcceleration()
	{
		auto o=RR::AllocateRRArray<double>(6);

			math::Vector3d a=get_entity()->WorldAngularAccel();
			math::Vector3d v=get_entity()->WorldLinearAccel();
			(*o)[0]=a.X(); (*o)[1]=a.Y(); (*o)[2]=a.Z();
			(*o)[3]=v.X(); (*o)[4]=v.Y(); (*o)[5]=v.Z();
			return o;
	}
	
	RR::RRArrayPtr<double > EntityImpl::get_RelativeAcceleration()
	{
		auto o=RR::AllocateRRArray<double>(6);

			math::Vector3d a=get_entity()->RelativeAngularAccel();
			math::Vector3d v=get_entity()->RelativeLinearAccel();
			(*o)[0]=a.X(); (*o)[1]=a.Y(); (*o)[2]=a.Z();
			(*o)[3]=v.X(); (*o)[4]=v.Y(); (*o)[5]=v.Z();
			return o;
	}
	
	RR::WirePtr<rrgz::PosePtr> EntityImpl::get_WorldPoseGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_WorldPoseGetWire;
	}
	void EntityImpl::set_WorldPoseGetWire(RR::WirePtr<rrgz::PosePtr> value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_WorldPoseGetWire) throw std::runtime_error("Already set");
		m_WorldPoseGetWire=value;
		m_WorldPoseGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<rrgz::PosePtr> >();
		m_WorldPoseGetWire_b->Init(m_WorldPoseGetWire);
	}

	RR::WirePtr<rrgz::PosePtr> EntityImpl::get_RelativePoseGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_RelativePoseGetWire;
	}
	void EntityImpl::set_RelativePoseGetWire(RR::WirePtr<rrgz::PosePtr> value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_RelativePoseGetWire) throw std::runtime_error("Already set");
		m_RelativePoseGetWire=value;
		m_RelativePoseGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<rrgz::PosePtr> >();
		m_RelativePoseGetWire_b->Init(m_RelativePoseGetWire);
	}

	RR::WirePtr<RR::RRArrayPtr<double > > EntityImpl::get_WorldVelocityGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_WorldVelocityGetWire;
	}
	void EntityImpl::set_WorldVelocityGetWire(RR::WirePtr<RR::RRArrayPtr<double > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_WorldVelocityGetWire) throw std::runtime_error("Already set");
		m_WorldVelocityGetWire=value;
		m_WorldVelocityGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR::RRArrayPtr<double > > >();
		m_WorldVelocityGetWire_b->Init(m_WorldVelocityGetWire);
	}

	RR::WirePtr<RR::RRArrayPtr<double > > EntityImpl::get_RelativeVelocityGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_RelativeVelocityGetWire;
	}
	void EntityImpl::set_RelativeVelocityGetWire(RR::WirePtr<RR::RRArrayPtr<double > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_RelativeVelocityGetWire) throw std::runtime_error("Already set");
		m_RelativeVelocityGetWire=value;
		m_RelativeVelocityGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR::RRArrayPtr<double > > >();
		m_RelativeVelocityGetWire_b->Init(m_RelativeVelocityGetWire);
	}

	RR::WirePtr<RR::RRArrayPtr<double > > EntityImpl::get_WorldAccelerationGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_WorldAccelerationGetWire;
	}
	void EntityImpl::set_WorldAccelerationGetWire(RR::WirePtr<RR::RRArrayPtr<double > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_WorldAccelerationGetWire) throw std::runtime_error("Already set");
		m_WorldAccelerationGetWire=value;
		m_WorldAccelerationGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR::RRArrayPtr<double > > >();
		m_WorldAccelerationGetWire_b->Init(m_WorldAccelerationGetWire);
	}

	RR::WirePtr<RR::RRArrayPtr<double > > EntityImpl::get_RelativeAccelerationGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_RelativeAccelerationGetWire;
	}
	void EntityImpl::set_RelativeAccelerationGetWire(RR::WirePtr<RR::RRArrayPtr<double > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_RelativeAccelerationGetWire) throw std::runtime_error("Already set");
		m_RelativeAccelerationGetWire=value;
		m_RelativeAccelerationGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR::RRArrayPtr<double> > >();
		m_RelativeAccelerationGetWire_b->Init(m_RelativeAccelerationGetWire);
	}

}
