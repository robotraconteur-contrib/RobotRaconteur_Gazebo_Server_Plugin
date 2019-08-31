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

#include "JointControllerImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	JointControllerImpl::JointControllerImpl(RR_SHARED_PTR<ModelImpl> model, physics::ModelPtr gz_model)
	{
		gz_controller=boost::make_shared<physics::JointController>(gz_model);

		this->gz_model=gz_model;
	}

	void JointControllerImpl::Init()
	{
		RR_WEAK_PTR<JointControllerImpl> j1=shared_from_this();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
						  boost::bind(&JointControllerImpl::OnUpdate, j1, _1));
	}

	RR::RRListPtr<RR::RRArray<char> > JointControllerImpl::get_JointNames()
	{
		auto o=RR::AllocateEmptyRRList<RR::RRArray<char> >();
		auto j_map=gz_controller->GetJoints();
		for (auto e=j_map.begin(); e!=j_map.end(); e++)
		{
			o->push_back(RR::stringToRRArray(e->second->GetScopedName(true)));
		}
		return o;
	}
	
	RR::RRMapPtr<std::string,pid::PIDParam > JointControllerImpl::get_PositionPIDs()
	{
		auto o=RR::AllocateEmptyRRMap<std::string,pid::PIDParam>();
		auto p1=gz_controller->GetPositionPIDs();
		for (auto e=p1.begin(); e!=p1.end(); e++)
		{
			pid::PIDParamPtr pid(new pid::PIDParam());
			pid->p=e->second.GetPGain();
			pid->d=e->second.GetDGain();
			pid->i=e->second.GetIGain();
			pid->imax=e->second.GetIMax();
			pid->imin=e->second.GetIMin();
			pid->cmdMax=e->second.GetCmdMax();
			pid->cmdMin=e->second.GetCmdMin();
			o->insert(std::make_pair(e->first,pid));
		}
		return o;
	}
	
	RR::RRMapPtr<std::string,pid::PIDParam> JointControllerImpl::get_VelocityPIDs()
	{
		auto o=RR::AllocateEmptyRRMap<std::string,pid::PIDParam>();
		auto p1=gz_controller->GetVelocityPIDs();
		for (auto e=p1.begin(); e!=p1.end(); e++)
		{
			pid::PIDParamPtr pid(new pid::PIDParam());
			pid->p=e->second.GetPGain();
			pid->d=e->second.GetDGain();
			pid->i=e->second.GetIGain();
			pid->imax=e->second.GetIMax();
			pid->imin=e->second.GetIMin();
			pid->cmdMax=e->second.GetCmdMax();
			pid->cmdMin=e->second.GetCmdMin();
			o->insert(std::make_pair(e->first,pid));
		}
		return o;
	}
	
	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_JointPositions(physics::JointControllerPtr& controller)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();		
		for (auto j : controller->GetJoints())
		{
			o->insert(std::make_pair(j.first, RR::ScalarToRRArray(j.second->Position(0))));
		}
		
		return o;
	}

	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_JointVelocities(physics::JointControllerPtr& controller)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();
		for (auto j : controller->GetJoints())
		{
			o->insert(std::make_pair(j.first, RR::ScalarToRRArray(j.second->GetVelocity(0))));
		}

		return o;
	}

	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_JointTargetPositions(physics::JointControllerPtr& controller)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();
		auto p1 = controller->GetPositions();
		for (auto e = p1.begin(); e != p1.end(); e++)
		{
			o->insert(std::make_pair(e->first, RR::ScalarToRRArray(e->second)));
		}
		return o;
	}
		
	static void _set_JointTargetPositions(physics::JointControllerPtr& controller, RR::RRMapPtr<std::string,RR::RRArray<double> > value)
	{
		RR_NULL_CHECK(value);
		for (auto e=value->begin(); e!=value->end(); e++)
		{
			RR_NULL_CHECK(e->second);
			controller->SetPositionTarget(e->first,RR::RRArrayToScalar(e->second));
		}
	}

	static RR::RRMapPtr<std::string,RR::RRArray<double> > _get_JointTargetVelocities(physics::JointControllerPtr& controller)
	{
		auto o=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto p1=controller->GetVelocities();
		for(auto e=p1.begin(); e!=p1.end(); e++)
		{
			o->insert(std::make_pair(e->first,RR::ScalarToRRArray(e->second)));
		}
		return o;
	}
	static void _set_JointTargetVelocities(physics::JointControllerPtr& controller, RR::RRMapPtr<std::string,RR::RRArray<double> > value)
	{
		RR_NULL_CHECK(value);
		for (auto e=value->begin(); e!=value->end(); e++)
		{
			RR_NULL_CHECK(e->second);
			controller->SetVelocityTarget(e->first,RR::RRArrayToScalar(e->second));
		}
	}

	RR::RRMapPtr<std::string,RR::RRArray<double> > _get_JointForces(physics::JointControllerPtr& controller)
	{
		auto o=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto p1=controller->GetForces();
		for(auto e=p1.begin(); e!=p1.end(); e++)
		{
			o->insert(std::make_pair(e->first,RR::ScalarToRRArray(e->second)));
		}
		return o;
	}
	
	void JointControllerImpl::AddJoint(const std::string& name)
	{
		physics::JointPtr j=get_model()->GetJoint(name);
		if (!j) throw std::invalid_argument("Invalid joint name");
		gz_controller->AddJoint(j);
	}

	void JointControllerImpl::SetPositionPID(const std::string& name, pid::PIDParamPtr pid)
	{
		RR_NULL_CHECK(pid);
		common::PID p(pid->p, pid->i, pid->d, pid->imax, pid->imin, pid->cmdMax, pid->cmdMin);
		gz_controller->SetPositionPID(name, p);
	}

	void JointControllerImpl::SetVelocityPID(const std::string& name, pid::PIDParamPtr pid)
	{
		RR_NULL_CHECK(pid);
		common::PID p(pid->p, pid->i, pid->d, pid->imax, pid->imin, pid->cmdMax, pid->cmdMin);
		gz_controller->SetVelocityPID(name, p);
	}
	
	void JointControllerImpl::set_JointPositions(RR::WirePtr<RR::RRMapPtr<std::string, RR::RRArray<double > > > value)
	{
		JointController_default_impl::set_JointPositions(value);
		boost::weak_ptr<JointControllerImpl> weak_this = shared_from_this();
		this->rrvar_JointPositions->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");				
				return _get_JointPositions(this_->gz_controller);
			}
		);
	}

	void JointControllerImpl::set_JointVelocities(RR::WirePtr<RR::RRMapPtr<std::string, RR::RRArray<double > > > value)
	{
		JointController_default_impl::set_JointVelocities(value);
		boost::weak_ptr<JointControllerImpl> weak_this = shared_from_this();
		this->rrvar_JointVelocities->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				return _get_JointVelocities(this_->gz_controller);
			}
		);
	}

	void JointControllerImpl::set_JointForces(RR::WirePtr<RR::RRMapPtr<std::string, RR::RRArray<double > > > value)
	{
		JointController_default_impl::set_JointForces(value);
		boost::weak_ptr<JointControllerImpl> weak_this = shared_from_this();
		this->rrvar_JointForces->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				return _get_JointForces(this_->gz_controller);
			}
		);
	}


	void JointControllerImpl::OnUpdate(RR_WEAK_PTR<JointControllerImpl> j, const common::UpdateInfo & _info)
	{
		RR_SHARED_PTR<JointControllerImpl> j1=j.lock();
		if (!j1) return;
		j1->OnUpdate1(_info);
	}

	void JointControllerImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > jointpositions_b;
		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > jointvelocities_b;
		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string, RR::RRArray<double> > > jointforces_b;

		RR::WireUnicastReceiverPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > targetpositions_b;
		RR::WireUnicastReceiverPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > targetvelocities_b;


		{
			boost::mutex::scoped_lock lock(this_lock);

			gz_controller->Update();

			jointpositions_b = rrvar_JointPositions;
			jointpositions_b = rrvar_JointVelocities;
			jointforces_b = rrvar_JointForces;
			targetpositions_b=rrvar_JointPositionsCommand;
			targetvelocities_b = rrvar_JointVelocitiesCommand;

		}

		auto o_pos=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto o_vel=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto o_f = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();

		for (auto j : gz_controller->GetJoints() | boost::adaptors::map_values)
		{			
			if (j->DOF()<1) continue;
			o_pos->insert(std::make_pair(j->GetName(),RR::ScalarToRRArray(j->Position(0))));
			o_vel->insert(std::make_pair(j->GetName(),RR::ScalarToRRArray(j->GetVelocity(0))));
			o_f->insert(std::make_pair(j->GetName(), RR::ScalarToRRArray(j->GetForce(0))));
		}

		if (jointpositions_b) jointpositions_b->SetOutValue(o_pos);
		if (jointvelocities_b) jointvelocities_b->SetOutValue(o_vel);
		if (jointforces_b) jointforces_b->SetOutValue(o_f);

		if (targetpositions_b)
		{
			RR::RRMapPtr<std::string,RR::RRArray<double> > targets;
			RR::TimeSpec ts;
			uint32_t ep;
			if (targetpositions_b->TryGetInValue(targets, ts, ep))
			{
				_set_JointTargetPositions(gz_controller, targets);
			}
		}

		if (targetvelocities_b)
		{
			RR::RRMapPtr<std::string, RR::RRArray<double> > targets;
			RR::TimeSpec ts;
			uint32_t ep;
			if (targetvelocities_b->TryGetInValue(targets, ts, ep))
			{
				_set_JointTargetVelocities(gz_controller, targets);
			}
		}
	}

	physics::ModelPtr JointControllerImpl::get_model()
	{
		physics::ModelPtr m=gz_model.lock();
		if (!m) throw std::runtime_error("Model has been released");
		return m;
	}

}
