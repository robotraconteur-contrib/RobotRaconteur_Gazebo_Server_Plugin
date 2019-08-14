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
	
	RR::RRMapPtr<std::string,rrgz::PIDParam > JointControllerImpl::get_PositionPIDs()
	{
		auto o=RR::AllocateEmptyRRMap<std::string,rrgz::PIDParam>();
		auto p1=gz_controller->GetPositionPIDs();
		for (auto e=p1.begin(); e!=p1.end(); e++)
		{
			rrgz::PIDParamPtr pid(new rrgz::PIDParam());
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
	
	RR::RRMapPtr<std::string,rrgz::PIDParam> JointControllerImpl::get_VelocityPIDs()
	{
		auto o=RR::AllocateEmptyRRMap<std::string,rrgz::PIDParam>();
		auto p1=gz_controller->GetVelocityPIDs();
		for (auto e=p1.begin(); e!=p1.end(); e++)
		{
			rrgz::PIDParamPtr pid(new rrgz::PIDParam());
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
	
	RR::RRMapPtr<std::string,RR::RRArray<double> > JointControllerImpl::get_JointTargetPositions()
	{
		auto o=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto p1=gz_controller->GetPositions();
		for(auto e=p1.begin(); e!=p1.end(); e++)
		{
			o->insert(std::make_pair(e->first,RR::ScalarToRRArray(e->second)));
		}
		return o;
	}
	void JointControllerImpl::set_JointTargetPositions(RR::RRMapPtr<std::string,RR::RRArray<double> > value)
	{
		RR_NULL_CHECK(value);
		for (auto e=value->begin(); e!=value->end(); e++)
		{
			RR_NULL_CHECK(e->second);
			gz_controller->SetPositionTarget(e->first,RR::RRArrayToScalar(e->second));
		}
	}

	RR::RRMapPtr<std::string,RR::RRArray<double> > JointControllerImpl::get_JointTargetVelocities()
	{
		auto o=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto p1=gz_controller->GetVelocities();
		for(auto e=p1.begin(); e!=p1.end(); e++)
		{
			o->insert(std::make_pair(e->first,RR::ScalarToRRArray(e->second)));
		}
		return o;
	}
	void JointControllerImpl::set_JointTargetVelocities(RR::RRMapPtr<std::string,RR::RRArray<double> > value)
	{
		RR_NULL_CHECK(value);
		for (auto e=value->begin(); e!=value->end(); e++)
		{
			RR_NULL_CHECK(e->second);
			gz_controller->SetVelocityTarget(e->first,RR::RRArrayToScalar(e->second));
		}
	}

	RR::RRMapPtr<std::string,RR::RRArray<double> > JointControllerImpl::get_JointForces()
	{
		auto o=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto p1=gz_controller->GetForces();
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
		boost::weak_ptr<physics::Joint> w_j=j;
		gz_joints.insert(std::make_pair(j->GetName(),w_j));
	}

	void JointControllerImpl::SetPositionPID(const std::string& name, rrgz::PIDParamPtr pid)
	{
		RR_NULL_CHECK(pid);
		common::PID p(pid->p, pid->i, pid->d, pid->imax, pid->imin, pid->cmdMax, pid->cmdMin);
		gz_controller->SetPositionPID(name, p);
	}

	void JointControllerImpl::SetVelocityPID(const std::string& name, rrgz::PIDParamPtr pid)
	{
		RR_NULL_CHECK(pid);
		common::PID p(pid->p, pid->i, pid->d, pid->imax, pid->imin, pid->cmdMax, pid->cmdMin);
		gz_controller->SetVelocityPID(name, p);
	}

	RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > JointControllerImpl::get_JointTargetPositionsSetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_JointTargetPositionsSetWire;
	}
	void JointControllerImpl::set_JointTargetPositionsSetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_JointTargetPositionsSetWire) throw std::runtime_error("Already set");
		m_JointTargetPositionsSetWire=value;
		RR_WEAK_PTR<JointControllerImpl> j=shared_from_this();
		m_JointTargetPositionsSetWire->SetWireConnectCallback(
				boost::bind(&JointControllerImpl::OnJointTargetPositionsSetWireConnect, j, _1));
	}

	void JointControllerImpl::OnJointTargetPositionsSetWireConnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > connection)
	{
		//TODO: This doesn't seem to work properly.
		throw std::runtime_error("Not implemented");
		RR_SHARED_PTR<JointControllerImpl> c1=c.lock();
		if (!c1) return;
		boost::mutex::scoped_lock lock(c1->this_lock);
		c1->m_JointTargetPositionsSetWire_c=connection;
		c1->m_JointTargetPositionsSetWire_c->SetWireConnectionClosedCallback(
				boost::bind(&JointControllerImpl::OnJointTargetPositionsSetWireDisconnect, c, _1));

	}
	void JointControllerImpl::OnJointTargetPositionsSetWireDisconnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > connection)
	{
		RR_SHARED_PTR<JointControllerImpl> c1=c.lock();
		if (!c1) return;
		boost::mutex::scoped_lock lock(c1->this_lock);
		c1->m_JointTargetPositionsSetWire_c.reset();
	}


	RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > JointControllerImpl::get_JointTargetVelocitiesSetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_JointTargetVelocitiesSetWire;
	}
	void JointControllerImpl::set_JointTargetVelocitiesSetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_JointTargetVelocitiesSetWire) throw std::runtime_error("Already set");
		m_JointTargetVelocitiesSetWire=value;
		RR_WEAK_PTR<JointControllerImpl> j=shared_from_this();
		m_JointTargetVelocitiesSetWire->SetWireConnectCallback(
		boost::bind(&JointControllerImpl::OnJointTargetVelocitiesSetWireConnect, j, _1));
	}

	void JointControllerImpl::OnJointTargetVelocitiesSetWireConnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > connection)
	{
		//TODO: This doesn't seem to work properly.
		throw std::runtime_error("Not implemented");
		RR_SHARED_PTR<JointControllerImpl> c1=c.lock();
		if (!c1) return;
		boost::mutex::scoped_lock lock(c1->this_lock);

		c1->m_JointTargetVelocitiesSetWire_c=connection;
		c1->m_JointTargetVelocitiesSetWire_c->SetWireConnectionClosedCallback(
				boost::bind(&JointControllerImpl::OnJointTargetVelocitiesSetWireDisconnect, c, _1));

	}
	void JointControllerImpl::OnJointTargetVelocitiesSetWireDisconnect(RR_WEAK_PTR<JointControllerImpl> c, RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > connection)
	{
		RR_SHARED_PTR<JointControllerImpl> c1=c.lock();
		if (!c1) return;
		boost::mutex::scoped_lock lock(c1->this_lock);
		c1->m_JointTargetVelocitiesSetWire_c.reset();

	}

	RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > JointControllerImpl::get_JointActualPositionsGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_JointActualPositionsGetWire;
	}
	void JointControllerImpl::set_JointActualPositionsGetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_JointActualPositionsGetWire) throw std::runtime_error("Already set");
		m_JointActualPositionsGetWire=value;
		m_JointActualPositionsGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR::RRMapPtr<std::string,RR::RRArray<double> > > >();
		m_JointActualPositionsGetWire_b->Init(m_JointActualPositionsGetWire);
	}

	RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > JointControllerImpl::get_JointActualVelocitiesGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_JointActualVelocitiesGetWire;
	}
	void JointControllerImpl::set_JointActualVelocitiesGetWire(RR::WirePtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_JointActualVelocitiesGetWire) throw std::runtime_error("Already set");
		m_JointActualVelocitiesGetWire=value;
		m_JointActualVelocitiesGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR::RRMapPtr<std::string,RR::RRArray<double> > > >();
		m_JointActualVelocitiesGetWire_b->Init(m_JointActualVelocitiesGetWire);
	}

	void JointControllerImpl::OnUpdate(RR_WEAK_PTR<JointControllerImpl> j, const common::UpdateInfo & _info)
	{
		RR_SHARED_PTR<JointControllerImpl> j1=j.lock();
		if (!j1) return;
		j1->OnUpdate1(_info);
	}

	void JointControllerImpl::OnUpdate1(const common::UpdateInfo & _info)
	{
		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > m_JointActualPositionsGetWire_b1;
		RR::WireBroadcasterPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > m_JointActualVelocitiesGetWire_b1;

		RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > m_JointTargetPositionsSetWire_c1;
		RR::WireConnectionPtr<RR::RRMapPtr<std::string,RR::RRArray<double> > > m_JointTargetVelocitiesSetWire_c1;


		{
			boost::mutex::scoped_lock lock(this_lock);

			gz_controller->Update();

			m_JointActualPositionsGetWire_b1=m_JointActualPositionsGetWire_b;
			m_JointActualVelocitiesGetWire_b1=m_JointActualVelocitiesGetWire_b;
			m_JointTargetPositionsSetWire_c1=m_JointTargetPositionsSetWire_c;
			m_JointTargetVelocitiesSetWire_c1=m_JointTargetPositionsSetWire_c;

		}

		auto o_pos=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();
		auto o_vel=RR::AllocateEmptyRRMap<std::string,RR::RRArray<double> >();

		for (auto e=gz_joints.begin(); e!=gz_joints.end(); e++)
		{
			RR_SHARED_PTR<physics::Joint> j=e->second.lock();
			if (!j) continue;
			if (j->DOF()<1) continue;
			o_pos->insert(std::make_pair(j->GetName(),RR::ScalarToRRArray(j->Position(0))));
			o_vel->insert(std::make_pair(j->GetName(),RR::ScalarToRRArray(j->GetVelocity(0))));
		}

		if (m_JointActualPositionsGetWire_b1) m_JointActualPositionsGetWire_b1->SetOutValue(o_pos);
		if (m_JointActualVelocitiesGetWire_b1) m_JointActualVelocitiesGetWire_b1->SetOutValue(o_vel);

		try
		{
			if (m_JointTargetPositionsSetWire_c1)
			{
				if (m_JointTargetPositionsSetWire_c1->GetInValueValid())
				{
					auto v=m_JointTargetPositionsSetWire_c1->GetInValue();
					if (v)
					{
						set_JointTargetPositions(v);
					}
				}

			}
		}
		catch (std::exception&) {}

		try
		{
			if (m_JointTargetVelocitiesSetWire_c1)
			{
				if (m_JointTargetVelocitiesSetWire_c1->GetInValueValid())
				{
					auto v=m_JointTargetVelocitiesSetWire_c1->GetInValue();
					if (v)
					{
						set_JointTargetVelocities(v);
					}
				}

			}
		}
		catch (std::exception&) {}

	}

	physics::ModelPtr JointControllerImpl::get_model()
	{
		physics::ModelPtr m=gz_model.lock();
		if (!m) throw std::runtime_error("Model has been released");
		return m;
	}

}
