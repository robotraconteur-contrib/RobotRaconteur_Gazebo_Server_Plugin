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

#include "JointControllerImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	JointControllerImpl::JointControllerImpl(RR_SHARED_PTR<ModelImpl> model, physics::ModelPtr gz_model)
	{
		gz_controller=boost::make_shared<physics::JointController>(gz_model);

		this->gz_model=gz_model;
	}

	RR::RRListPtr<RR::RRArray<char> > JointControllerImpl::get_joint_names()
	{
		auto o=RR::AllocateEmptyRRList<RR::RRArray<char> >();
		auto j_map=gz_controller->GetJoints();
		for (auto e=j_map.begin(); e!=j_map.end(); e++)
		{
			o->push_back(RR::stringToRRArray(e->second->GetScopedName(true)));
		}
		return o;
	}
	
	RR::RRMapPtr<std::string,pid::PIDParam > JointControllerImpl::get_position_pid()
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
			pid->cmd_max=e->second.GetCmdMax();
			pid->cmd_min=e->second.GetCmdMin();
			o->insert(std::make_pair(e->first,pid));
		}
		return o;
	}
	
	RR::RRMapPtr<std::string,pid::PIDParam> JointControllerImpl::get_velocity_pid()
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
			pid->cmd_max=e->second.GetCmdMax();
			pid->cmd_min=e->second.GetCmdMin();
			o->insert(std::make_pair(e->first,pid));
		}
		return o;
	}
	
	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_joint_position(physics::JointControllerPtr& controller)
	{
		auto o = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();		
		for (auto j : controller->GetJoints())
		{
			o->insert(std::make_pair(j.first, RR::ScalarToRRArray(j.second->Position(0))));
		}
		
		return o;
	}

	static RR::RRMapPtr<std::string, RR::RRArray<double> > _get_joint_velocity(physics::JointControllerPtr& controller)
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

	RR::RRMapPtr<std::string,RR::RRArray<double> > _get_joint_forces(physics::JointControllerPtr& controller)
	{
		auto o=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto p1=controller->GetForces();
		for(auto e=p1.begin(); e!=p1.end(); e++)
		{
			o->insert(std::make_pair(e->first,RR::ScalarToRRArray(e->second)));
		}
		return o;
	}
	
	void JointControllerImpl::add_joint(const std::string& name)
	{
		physics::JointPtr j=get_model()->GetJoint(name);
		if (!j) throw std::invalid_argument("Invalid joint name");
		gz_controller->AddJoint(j);
	}

	void JointControllerImpl::setf_position_pid(const std::string& name, pid::PIDParamPtr pid)
	{
		RR_NULL_CHECK(pid);
		common::PID p(pid->p, pid->i, pid->d, pid->imax, pid->imin, pid->cmd_max, pid->cmd_min);
		gz_controller->SetPositionPID(name, p);
	}

	void JointControllerImpl::setf_velocity_pid(const std::string& name, pid::PIDParamPtr pid)
	{
		RR_NULL_CHECK(pid);
		common::PID p(pid->p, pid->i, pid->d, pid->imax, pid->imin, pid->cmd_max, pid->cmd_min);
		gz_controller->SetVelocityPID(name, p);
	}
	
	void JointControllerImpl::OnUpdate(RR_WEAK_PTR<JointControllerImpl> j, const common::UpdateInfo & _info)
	{
		RR_SHARED_PTR<JointControllerImpl> j1=j.lock();
		if (!j1) return;
		j1->OnUpdate1(_info);
	}

	void JointControllerImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		RR::BroadcastDownsamplerStep step(rr_downsampler);

		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > jointpositions_b;
		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > jointvelocities_b;
		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string, RR::RRArray<double> > > jointforces_b;

		RR::WireUnicastReceiverPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > targetpositions_b;
		RR::WireUnicastReceiverPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > targetvelocities_b;


		{
			boost::mutex::scoped_lock lock(this_lock);

			gz_controller->Update();

			jointpositions_b = rrvar_joint_position;
			jointpositions_b = rrvar_joint_velocity;
			jointforces_b = rrvar_joint_forces;
			targetpositions_b=rrvar_joint_position_command;
			targetvelocities_b = rrvar_joint_velocity_command;

		}

		auto o_pos=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto o_vel=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto o_f = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double> >();

		auto gz_controller_joints = gz_controller->GetJoints();
		for (auto j : gz_controller_joints | boost::adaptors::map_values)
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

	void JointControllerImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		rr_path = service_path;

		rr_downsampler = boost::make_shared<RR::BroadcastDownsampler>();
		rr_downsampler->Init(context.lock(),9);

		rr_downsampler->AddWireBroadcaster(rrvar_joint_position);
		rr_downsampler->AddWireBroadcaster(rrvar_joint_velocity);
		rr_downsampler->AddWireBroadcaster(rrvar_joint_forces);

		RR_WEAK_PTR<JointControllerImpl> weak_this=shared_from_this();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
						  boost::bind(&JointControllerImpl::OnUpdate, weak_this, _1));

		this->rrvar_joint_position->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");				
				return _get_joint_position(this_->gz_controller);
			}
		);

		this->rrvar_joint_velocity->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				return _get_joint_velocity(this_->gz_controller);
			}
		);

		this->rrvar_joint_forces->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Joint has been released");
				return _get_joint_forces(this_->gz_controller);
			}
		);



	}

	std::string JointControllerImpl::RRPath()
	{
		return rr_path;
	}

	com::robotraconteur::device::isoch::IsochInfoPtr JointControllerImpl::get_isoch_info()
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

	uint32_t JointControllerImpl::get_isoch_downsample()
	{
		return rr_downsampler->GetClientDownsample(RR::ServerEndpoint::GetCurrentEndpoint()->GetLocalEndpoint());
	}

	void JointControllerImpl::set_isoch_downsample(uint32_t value)
	{
		return rr_downsampler->SetClientDownsample(RR::ServerEndpoint::GetCurrentEndpoint()->GetLocalEndpoint(), value);
	}

}
