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

#include "KinematicJointControllerImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	KinematicJointControllerImpl::KinematicJointControllerImpl(RR_SHARED_PTR<ModelImpl> model, physics::ModelPtr gz_model)
	{
		
		this->gz_model=gz_model;
	}

	RR::RRListPtr<RR::RRArray<char> > KinematicJointControllerImpl::get_joint_names()
	{
		auto o=RR::AllocateEmptyRRList<RR::RRArray<char> >();
		
		for (auto e : joints | boost::adaptors::map_values)
		{
			auto e2 = e.lock();
			if (!e2) continue;
			o->push_back(RR::stringToRRArray(e2->GetName()));
		}
		return o;
	}
	
	RR::RRMapPtr<std::string,pid::PIDParam > KinematicJointControllerImpl::get_position_pid()
	{
		throw RR::NotImplementedException("Not implemented for KinematicJointController");
	}
	
	RR::RRMapPtr<std::string,pid::PIDParam> KinematicJointControllerImpl::get_velocity_pid()
	{
		throw RR::NotImplementedException("Not implemented for KinematicJointController");
	}
	
	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_joint_position(std::map<std::string,boost::weak_ptr<physics::Joint> >& joints)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();
		for (auto j : joints)
		{
			auto j2 = j.second.lock();
			if (!j2) continue;
			o->insert(std::make_pair(j2->GetName(), RR::ScalarToRRArray(j2->Position(0))));
		}

		return o;
	}

	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_joint_velocity(std::map<std::string,boost::weak_ptr<physics::Joint> >& joints)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();
		for (auto j : joints)
		{
			auto j2 = j.second.lock();
			if (!j2) continue;
			o->insert(std::make_pair(j2->GetName(), RR::ScalarToRRArray(j2->GetVelocity(0))));
		}

		return o;
	}

	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_JointTargetPositions(std::map<std::string,boost::weak_ptr<physics::Joint> >& joints, RR::RRMapPtr<std::string,RR::RRArray<double> >& joint_targets)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();
		if (joint_targets){
			for (auto j : joints)
			{
				auto t = joint_targets->find(j.first);
				if (t == joint_targets->end()) continue;
				if (t->second->size() != 1) continue;
				o->insert(std::make_pair(t->first, t->second));
			}
		}
		return o;
	}
		
	static RR::RRMapPtr<std::string,RR::RRArray<double> > _get_JointTargetVelocities(std::map<std::string,boost::weak_ptr<physics::Joint> >& joints, RR::RRMapPtr<std::string,RR::RRArray<double> >& joint_velocities)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();
		if (joint_velocities){
			for (auto j : joints)
			{
				auto t = joint_velocities->find(j.first);
				if (t == joint_velocities->end()) continue;
				if (t->second->size() != 1) continue;
				o->insert(std::make_pair(t->first, t->second));
			}
		}
		return o;
	}
	
		
	void KinematicJointControllerImpl::add_joint(const std::string& name)
	{
		physics::JointPtr j=get_model()->GetJoint(name);
		if (!j) throw std::invalid_argument("Invalid joint name");
		joints.insert(std::make_pair(name,j));
	}

	void KinematicJointControllerImpl::setf_position_pid(const std::string& name, pid::PIDParamPtr pid)
	{
		throw RR::NotImplementedException("Not implemented for KinematicJointController");
	}

	void KinematicJointControllerImpl::setf_velocity_pid(const std::string& name, pid::PIDParamPtr pid)
	{
		throw RR::NotImplementedException("Not implemented for KinematicJointController");
	}
	
	void KinematicJointControllerImpl::OnUpdate(RR_WEAK_PTR<KinematicJointControllerImpl> j, const common::UpdateInfo & _info)
	{
		RR_SHARED_PTR<KinematicJointControllerImpl> j1=j.lock();
		if (!j1) return;
		j1->OnUpdate1(_info);
	}

	void KinematicJointControllerImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		RR::BroadcastDownsamplerStep step(rr_downsampler);

		auto o_pos=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto o_vel=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		
		for (auto j1 : joints | boost::adaptors::map_values)
		{			
			auto j = j1.lock();
			if (!j) continue;
			if (j->DOF()<1) continue;
			o_pos->insert(std::make_pair(j->GetName(),RR::ScalarToRRArray(j->Position(0))));
			o_vel->insert(std::make_pair(j->GetName(),RR::ScalarToRRArray(j->GetVelocity(0))));
		}

		rrvar_joint_position->SetOutValue(o_pos);
		rrvar_joint_velocity->SetOutValue(o_vel);
		

		if (joint_targets)
		{
			for (auto c : *joint_targets)
			{				
				auto e = joints.find(c.first);
				if (e==joints.end()) continue;
				if (c.second->size() != 1) continue;
				auto j = e->second.lock();
				if (!j) continue;
				j->SetPosition(0,c.second->at(0));
			}
		}

		if (joint_velocities)
		{
			for (auto c : *joint_velocities)
			{				
				auto e = joints.find(c.first);
				if (e==joints.end()) continue;
				if (c.second->size() != 1) continue;
				auto j = e->second.lock();
				if (!j) continue;
				j->SetVelocity(0,c.second->at(0));
			}
		}
	}

	physics::ModelPtr KinematicJointControllerImpl::get_model()
	{
		physics::ModelPtr m=gz_model.lock();
		if (!m) throw std::runtime_error("Model has been released");
		return m;
	}

	void KinematicJointControllerImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		rr_path = service_path;

		rr_downsampler = boost::make_shared<RR::BroadcastDownsampler>();
		rr_downsampler->Init(context.lock(),9);

		rr_downsampler->AddWireBroadcaster(rrvar_joint_position);
		rr_downsampler->AddWireBroadcaster(rrvar_joint_velocity);
		rr_downsampler->AddWireBroadcaster(rrvar_joint_forces);

		RR_WEAK_PTR<KinematicJointControllerImpl> weak_this=shared_from_this();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
						  boost::bind(&KinematicJointControllerImpl::OnUpdate, weak_this, _1));

		this->rrvar_joint_position->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");				
				return _get_joint_position(this_->joints);
			}
		);

		this->rrvar_joint_velocity->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				return _get_joint_velocity(this_->joints);
			}
		);

		this->rrvar_joint_forces->GetWire()->SetPeekInValueCallback(
			[](uint32_t ep) -> RR::RRMapPtr<std::string,RR::RRArray<double> >
			{
				throw RR::NotImplementedException("Not implemented for KinematicJointController");
			}
		);
		this->rrvar_joint_forces->GetWire()->SetWireConnectCallback(
			[](RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > w)
			{
				throw RR::NotImplementedException("Not implemented for KinematicJointController");
			}
		);

		this->rrvar_joint_position_command->GetWire()->SetPeekOutValueCallback(
			[weak_this](uint32_t ep)
			{
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				if (!this_->joint_targets)
				{
					return RR::RRMapPtr<std::string,RR::RRArray<double> >();
				}
				return this_->joint_targets;
			}
		);

		this->rrvar_joint_position_command->GetWire()->SetPokeOutValueCallback(
			[weak_this](RR::RRMapPtr<std::string,RR::RRArray<double> > target, const RR::TimeSpec& ts, uint32_t ep)
			{
				RR::rr_null_check(target);
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				for (auto t : *target)
				{
					if (this_->joints.find(t.first) == this_->joints.end())
					{
						throw RR::InvalidArgumentException("Invalid joint name: " + t.first);
					}

					if (RR::rr_null_check(t.second)->size() != 1)
					{
						throw RR::InvalidArgumentException("Expected scalar target for joint: " + t.first);
					}
				}

				this_->joint_targets = target;
				this_->joint_velocities.reset();				
			}
		);

		this->rrvar_joint_position_command->InValueChanged.connect(
			[weak_this](RR::RRMapPtr<std::string,RR::RRArray<double> > target, const RR::TimeSpec& ts, uint32_t ep)
			{
				if(!target) return;
				auto this_ = weak_this.lock();
				if (!this_) return;
				for (auto t : *target)
				{
					if (this_->joints.find(t.first) == this_->joints.end())
					{
						return;
					}
					if (!t.second) 
					{
						return;
					}
					if (t.second->size() != 1)
					{
						return;
					}
				}

				this_->joint_targets = target;
				this_->joint_velocities.reset();				
			}
		);

	}

	std::string KinematicJointControllerImpl::RRPath()
	{
		return rr_path;
	}

	com::robotraconteur::device::isoch::IsochInfoPtr KinematicJointControllerImpl::get_isoch_info()
	{
		com::robotraconteur::device::isoch::IsochInfoPtr ret(new com::robotraconteur::device::isoch::IsochInfo());
		auto world = get_model()->GetWorld();
		common::Time start_time = world->StartTime();
		ret->isoch_epoch.seconds = start_time.sec;
		ret->isoch_epoch.nanoseconds = start_time.nsec;
		ret->max_downsample = 100;
		ret->update_rate = world->Physics()->GetRealTimeUpdateRate();

		return ret;
	}

	uint32_t KinematicJointControllerImpl::get_isoch_downsample()
	{
		return rr_downsampler->GetClientDownsample(RR::ServerEndpoint::GetCurrentEndpoint()->GetLocalEndpoint());
	}

	void KinematicJointControllerImpl::set_isoch_downsample(uint32_t value)
	{
		return rr_downsampler->SetClientDownsample(RR::ServerEndpoint::GetCurrentEndpoint()->GetLocalEndpoint(), value);
	}

}
