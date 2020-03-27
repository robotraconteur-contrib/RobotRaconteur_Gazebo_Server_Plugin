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

		auto j=gz_joint.lock();
		if (!j)
		{
			auto context = rr_context.lock();
			if (context)
			{
				try
				{
				context->ReleaseServicePath(rr_path);
				}
				catch (std::exception&) {}
			}
			return;
		}

		size_t axis_count;		
		
		axis_count = j->DOF();
				
		rrvar_axes_position->SetOutValue(_get_axes_Positions(j));
				
		rrvar_axes_velocity->SetOutValue(_get_axes_velocities(j));
		
		rrvar_axes_force->SetOutValue(_get_axes_force(j));
		
		rrvar_force_torque->SetOutValue(_get_force_torque(j));
		

		
		// TODO: timeout on axes force command
		RR::RRArrayPtr<double> force;
		RR::TimeSpec ts;
		uint32_t ep;
		if(rrvar_apply_axes_force->TryGetInValue(force, ts, ep))
		{
			unsigned int dof = j->DOF();
			for (unsigned int i=0; i<force->size(); i++)
			{
				if (i < dof)
				{
					j->SetForce(i, (*force)[i]);
				}
			}
		}
			
		
	}

	physics::JointPtr JointImpl::get_joint()
	{
		physics::JointPtr j=gz_joint.lock();
		if (!j) throw std::runtime_error("Joint has been released");
		return j;
	}

	std::string JointImpl::get_name()
	{
		return get_joint()->GetName();
	}
	
	std::string JointImpl::get_scoped_name()
	{
		return get_joint()->GetScopedName(true);
	}
	
	std::string JointImpl::get_parent_link_name()
	{
		return get_joint()->GetParent()->GetName();
	}
	
	std::string JointImpl::get_child_link_name()
	{
		return get_joint()->GetChild()->GetName();
	}
	
	uint32_t JointImpl::get_dof()
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
	
	RR::RRNamedArrayPtr<geometry::Vector3> JointImpl::getf_global_axes()
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

	RR::RRNamedArrayPtr<geometry::Vector3> JointImpl::getf_local_axes()
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
		
	void JointImpl::setf_axis_position(uint32_t axis, double value)
	{
		physics::JointPtr j = get_joint();
		int axis_count = j->DOF();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		j->SetPosition(axis, value);
	}

	void JointImpl::setf_axis_velocity(uint32_t axis, double value)
	{
		physics::JointPtr j = get_joint();
		int axis_count = j->DOF();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		j->SetVelocity(axis, value);
	}

	void JointImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		rr_context = context;
		rr_path = service_path;

		RR_WEAK_PTR<JointImpl> weak_this = shared_from_this();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&JointImpl::OnUpdate, weak_this, _1));

		this->rrvar_axes_position->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();			
				return _get_axes_Positions(j);
			}
		);

		this->rrvar_axes_velocity->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();
				return _get_axes_velocities(j);
			}
		);

		this->rrvar_axes_force->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();
				return _get_axes_force(j);
			}
		);

		this->rrvar_force_torque->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				auto j = this_->get_joint();
				return _get_force_torque(j);
			}
		);

	}


}
