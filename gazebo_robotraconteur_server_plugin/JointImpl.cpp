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

#include "JointImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	JointImpl::JointImpl(physics::JointPtr j)
	{
		gz_joint=j;
		link_name=j->GetParent()->GetName();
		model_name=j->GetParent()->GetParentModel()->GetName();
		gz_world=j->GetParent()->GetParentModel()->GetWorld();
		
		axes_forces=RR::AllocateRRArray<double>(j->DOF());
		j->SetProvideFeedback(true);
	}

	void JointImpl::Init()
	{
		RR_WEAK_PTR<JointImpl> j1=shared_from_this();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		          boost::bind(&JointImpl::OnUpdate, j1, _1));
	}

	static geometry::Vector3 gz_vector_to_rr_vector(const ignition::math::Vector3d& v)
	{
		geometry::Vector3 o;
		o.s.x = v.X();
		o.s.y = v.Y();
		o.s.z = v.Z();
		return o;
	}

	static RR::RRArrayPtr<double> _get_axes_Positions(const physics::JointPtr& j)
	{
		size_t axis_count = j->DOF();
		auto o = RR::AllocateRRArray<double>(axis_count);
		for (int32_t i = 0; i < axis_count; i++)
		{
			o->at(i) = j->Position(i);
		}
		return o;
	}

	static RR::RRArrayPtr<double> _get_axes_velocities(const physics::JointPtr& j)
	{
		size_t axis_count = j->DOF();
		auto o = RR::AllocateRRArray<double>(axis_count);
		for (int32_t i = 0; i < axis_count; i++)
		{
			o->at(i) = j->GetVelocity(i);
		}
		return o;
	}

	static RR::RRArrayPtr<double> _get_axes_force(const physics::JointPtr& j)
	{
		size_t axis_count = j->DOF();
		auto o = RR::AllocateRRArray<double>(axis_count);
		for (int32_t i = 0; i < axis_count; i++)
		{
			o->at(i) = j->GetForce(i);
		}
		return o;
	}

	static geometry::Wrench gz_to_wrench(const ignition::math::Vector3d& m, const ignition::math::Vector3d& f)
	{
		geometry::Wrench o;
		o.s.torque.s.x = m.X();
		o.s.torque.s.y = m.Y();
		o.s.torque.s.z = m.Z();
		o.s.force.s.x = f.X();
		o.s.force.s.y = f.Y();
		o.s.force.s.z = f.Z();
		return o;
	}

	static rrgz::JointWrench _get_force_torque(const physics::JointPtr& j)
	{
		rrgz::JointWrench o;

		physics::JointWrench a = j->GetForceTorque(0);

		o.s.body1_wrench = gz_to_wrench(a.body1Torque, a.body1Force);
		o.s.body2_wrench = gz_to_wrench(a.body2Torque, a.body2Force);

		return o;
	}


	void JointImpl::OnUpdate(RR_WEAK_PTR<JointImpl> j, const common::UpdateInfo & _info)
    {
		RR_SHARED_PTR<JointImpl> j1=j.lock();
		if (!j1) return;
		j1->OnUpdate1(_info);
    }

	void JointImpl::OnUpdate1(const common::UpdateInfo & _info)
	{

		//std::cout << _info.simTime.Double() << std::endl;


		RR::WireBroadcasterPtr<RR::RRArrayPtr<double> > axesPositions_b;
		RR::WireBroadcasterPtr<RR::RRArrayPtr<double> > axesvel_b;
		RR::WireBroadcasterPtr<RR::RRArrayPtr<double> > axesforce_b;
		RR::WireBroadcasterPtr<rrgz::JointWrench> ft_b;
				
		size_t axis_count;		
		physics::JointPtr j=get_joint();
		axis_count = j->DOF();
		{
			boost::mutex::scoped_lock lock(this_lock);

			
			axesPositions_b=rrvar_AxesPositions;
			axesvel_b=rrvar_AxesVelocities;
			axesforce_b = rrvar_AxesForce;
			ft_b=rrvar_ForceTorque;
		}
		
		if (axesPositions_b)
		{
			axesPositions_b->SetOutValue(_get_axes_Positions(j));
		}
		
		if (axesvel_b)
		{
			axesvel_b->SetOutValue(_get_axes_velocities(j));
		}

		if (axesforce_b)
		{
			axesforce_b->SetOutValue(_get_axes_force(j));
		}

		if (ft_b)
		{
			ft_b->SetOutValue(_get_force_torque(j));
		}		
	}

	physics::JointPtr JointImpl::get_joint()
	{
		physics::JointPtr j=gz_joint.lock();
		if (!j) throw std::runtime_error("Joint has been released");
		return j;
	}

	std::string JointImpl::get_Name()
	{
		return get_joint()->GetName();
	}
	
	std::string JointImpl::get_ScopedName()
	{
		return get_joint()->GetScopedName(true);
	}
	
	std::string JointImpl::get_ParentLinkName()
	{
		return get_joint()->GetParent()->GetName();
	}
	
	std::string JointImpl::get_ChildLinkName()
	{
		return get_joint()->GetChild()->GetName();
	}
	
	uint32_t JointImpl::get_AxisCount()
	{
		return get_joint()->DOF();
	}
	
	
	static void _set_axes_Positions(RR::RRMapPtr<int32_t,RR::RRArray<double > > value, const physics::JointPtr& j)
	{
		RR_NULL_CHECK(value);		
		int axis_count=j->DOF();
		for (auto e=value->begin(); e!=value->end(); e++)
		{
			RR_NULL_CHECK(e->second);
			if (e->first > axis_count)
			{
				throw std::invalid_argument("Invalid axis");
			}
			double v=0;
			v=RR::RRArrayToScalar(e->second);
			j->SetPosition((unsigned int)e->first, v);
		}
	}

	
	static void _set_axes_force(RR::RRMapPtr<int32_t,RR::RRArray<double > > value, const physics::JointPtr& j)
	{
		RR_NULL_CHECK(value);		
		int axis_count=j->DOF();
		for (auto e=value->begin(); e!=value->end(); e++)
		{
			RR_NULL_CHECK(e->second);
			if (e->first > axis_count)
			{
				throw std::invalid_argument("Invalid axis");
			}
			double v=0;
			v=RR::RRArrayToScalar(e->second);
			j->SetForce((unsigned int)e->first, v);
		}
	}
	
	RR::RRNamedArrayPtr<geometry::Vector3> JointImpl::GetGlobalAxes()
	{		
		physics::JointPtr j=get_joint();
		size_t axis_count=j->DOF();
		RR::RRNamedArrayPtr<geometry::Vector3> o = RR::AllocateEmptyRRNamedArray<geometry::Vector3>(axis_count);
		for (size_t i = 0; i < axis_count; i++)
		{
			auto v = j->GlobalAxis(i);
			o->at(i) = gz_vector_to_rr_vector(v);

		}		
		return o;
	}

	RR::RRNamedArrayPtr<geometry::Vector3> JointImpl::GetLocalAxes()
	{
		physics::JointPtr j = get_joint();
		size_t axis_count = j->DOF();
		RR::RRNamedArrayPtr<geometry::Vector3> o = RR::AllocateEmptyRRNamedArray<geometry::Vector3>(axis_count);
		for (size_t i = 0; i < axis_count; i++)
		{
			auto v = j->LocalAxis(i);
			o->at(i) = gz_vector_to_rr_vector(v);

		}
		return o;
	}

	void JointImpl::set_AxesPositions(RR::WirePtr<RR::RRArrayPtr<double> > value)
	{
		Joint_default_impl::set_AxesPositions(value);
		boost::weak_ptr<JointImpl> weak_this = shared_from_this();
		this->rrvar_AxesPositions->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();			
				return _get_axes_Positions(j);
			}
		);
	}
	
	void JointImpl::set_AxesVelocities(RR::WirePtr<RR::RRArrayPtr<double> > value)
	{
		Joint_default_impl::set_AxesVelocities(value);
		boost::weak_ptr<JointImpl> weak_this = shared_from_this();
		this->rrvar_AxesVelocities->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();
				return _get_axes_velocities(j);
			}
		);
	}
	
	void JointImpl::set_AxesForce(RR::WirePtr<RR::RRArrayPtr<double> > value)
	{
		Joint_default_impl::set_AxesForce(value);
		boost::weak_ptr<JointImpl> weak_this = shared_from_this();
		this->rrvar_AxesForce->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();
				return _get_axes_force(j);
			}
		);
	}
		
	void JointImpl::set_ForceTorque(RR::WirePtr<rrgz::JointWrench> value)
	{
		Joint_default_impl::set_ForceTorque(value);
		boost::weak_ptr<JointImpl> weak_this = shared_from_this();
		this->rrvar_ForceTorque->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();
				return _get_force_torque(j);
			}
		);
	}

	void JointImpl::SetAxisPosition(uint32_t axis, double value)
	{
		physics::JointPtr j = get_joint();
		int axis_count = j->DOF();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		j->SetPosition(axis, value);
	}

	void JointImpl::SetAxisVelocity(uint32_t axis, double value)
	{
		physics::JointPtr j = get_joint();
		int axis_count = j->DOF();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		j->SetVelocity(axis, value);
	}

}
