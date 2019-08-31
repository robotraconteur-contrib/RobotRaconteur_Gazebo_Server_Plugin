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
	static geometry::Pose gz_pose_to_rr_pose(const math::Pose3d& p)
	{
		geometry::Pose o;
		o.s.position.s.x = p.Pos().X();
		o.s.position.s.y = p.Pos().Y();
		o.s.position.s.z = p.Pos().Z();
		o.s.orientation.s.w = p.Rot().W();
		o.s.orientation.s.x = p.Rot().X();
		o.s.orientation.s.y = p.Rot().Y();
		o.s.orientation.s.z = p.Rot().Z();
		return o;
	}

	static math::Pose3d rr_pose_to_gz_pose(const geometry::Pose& value)
	{
		math::Vector3d v(value.s.position.s.x, value.s.position.s.y, value.s.position.s.z);
		math::Quaterniond q(value.s.orientation.s.w, value.s.orientation.s.x,
			value.s.orientation.s.y, value.s.orientation.s.z);
		math::Pose3d p(v, q);
		return p;
	}

	static geometry::SpatialVelocity gz_to_spatial_velocity(const math::Vector3d& a, const math::Vector3d& v)
	{
		geometry::SpatialVelocity o;
		o.s.linear.s.x = v.X();
		o.s.linear.s.y = v.Y();
		o.s.linear.s.z = v.Z();
		o.s.angular.s.x = a.X();
		o.s.angular.s.y = a.Y();
		o.s.angular.s.z = a.Z();
		return o;
	}

	static geometry::SpatialAcceleration gz_to_spatial_acceleration(const math::Vector3d& a, const math::Vector3d& v)
	{
		geometry::SpatialAcceleration o;
		o.s.linear.s.x = v.X();
		o.s.linear.s.y = v.Y();
		o.s.linear.s.z = v.Z();
		o.s.angular.s.x = a.X();
		o.s.angular.s.y = a.Y();
		o.s.angular.s.z = a.Z();
		return o;
	}

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
		RR::WireBroadcasterPtr<geometry::Pose> wpose;
		RR::WireBroadcasterPtr<geometry::Pose> rpose;
		RR::WireBroadcasterPtr<geometry::SpatialVelocity> wvel;
		RR::WireBroadcasterPtr<geometry::SpatialVelocity> rvel;
		RR::WireBroadcasterPtr<geometry::SpatialAcceleration> wacc;
		RR::WireBroadcasterPtr<geometry::SpatialAcceleration> racc;
		{
			boost::mutex::scoped_lock lock(this_lock);
			wpose=rrvar_WorldPose;
			rpose=rrvar_RelativePose;
			wvel=rrvar_WorldVelocity;
			rvel=rrvar_RelativeVelocity;
			wacc=rrvar_WorldAcceleration;
			racc=rrvar_RelativeAcceleration;

		}

		auto e = get_entity();

		auto wpose1=gz_pose_to_rr_pose(e->WorldPose());
		auto rpose1= gz_pose_to_rr_pose(e->RelativePose());
		auto wvel1=gz_to_spatial_velocity(e->WorldAngularVel(), e->WorldLinearVel());
		auto rvel1=gz_to_spatial_velocity(e->RelativeAngularVel(), e->RelativeLinearVel());
		auto wacc1=gz_to_spatial_acceleration(e->WorldAngularAccel(), e->WorldLinearAccel());
		auto racc1= gz_to_spatial_acceleration(e->RelativeAngularAccel(), e->RelativeLinearAccel());

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

	void EntityImpl::set_WorldPose(RR::WirePtr<geometry::Pose> value)
	{
		Entity_default_impl::set_WorldPose(value);
		boost::weak_ptr<EntityImpl> weak_this = shared_from_this();
		this->rrvar_WorldPose->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto p = this_->get_entity()->WorldPose();				
				return gz_pose_to_rr_pose(p);
			}
		);
	}

	void EntityImpl::SetWorldPose(const geometry::Pose& value)
	{		
		get_entity()->SetWorldPose(rr_pose_to_gz_pose(value), true, true);
	}

	void EntityImpl::set_RelativePose(RR::WirePtr<geometry::Pose> value)
	{
		Entity_default_impl::set_RelativePose(value);
		boost::weak_ptr<EntityImpl> weak_this = shared_from_this();
		this->rrvar_RelativePose->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto p = this_->get_entity()->RelativePose();
				return gz_pose_to_rr_pose(p);
			}
		);
	}

	void EntityImpl::SetRelativePose(const geometry::Pose& value)
	{
		get_entity()->SetRelativePose(rr_pose_to_gz_pose(value), true, true);
	}

	void EntityImpl::set_WorldVelocity(RR::WirePtr<geometry::SpatialVelocity> value)
	{
		Entity_default_impl::set_WorldVelocity(value);
		boost::weak_ptr<EntityImpl> weak_this = shared_from_this();
		this->rrvar_WorldVelocity->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto a = this_->get_entity()->WorldAngularVel();
				auto v = this_->get_entity()->WorldLinearVel();
				return gz_to_spatial_velocity(a, v);
			}
		);
	}

	void EntityImpl::set_RelativeVelocity(RR::WirePtr<geometry::SpatialVelocity> value)
	{
		Entity_default_impl::set_RelativeVelocity(value);
		boost::weak_ptr<EntityImpl> weak_this = shared_from_this();
		this->rrvar_RelativeVelocity->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto a = this_->get_entity()->RelativeAngularVel();
				auto v = this_->get_entity()->RelativeLinearVel();
				return gz_to_spatial_velocity(a, v);
			}
		);
	}
	
	void EntityImpl::set_WorldAcceleration(RR::WirePtr<geometry::SpatialAcceleration> value)
	{
		Entity_default_impl::set_WorldAcceleration(value);
		boost::weak_ptr<EntityImpl> weak_this = shared_from_this();
		this->rrvar_WorldAcceleration->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto a = this_->get_entity()->WorldAngularAccel();
				auto v = this_->get_entity()->WorldLinearAccel();
				return gz_to_spatial_acceleration(a, v);
			}
		);
	}
			
	void EntityImpl::set_RelativeAcceleration(RR::WirePtr<geometry::SpatialAcceleration> value)
	{
		Entity_default_impl::set_RelativeAcceleration(value);
		boost::weak_ptr<EntityImpl> weak_this = shared_from_this();
		this->rrvar_RelativeAcceleration->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto a = this_->get_entity()->RelativeAngularAccel();
				auto v = this_->get_entity()->RelativeLinearAccel();
				return gz_to_spatial_acceleration(a, v);
			}
		);
	}
}
