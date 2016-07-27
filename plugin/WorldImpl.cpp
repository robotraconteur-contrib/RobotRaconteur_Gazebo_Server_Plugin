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

#include "WorldImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
WorldImpl::WorldImpl(physics::WorldPtr w)
{
	gz_world=w;
}

void WorldImpl::Init(const std::string& rr_path)
{
	RR_WEAK_PTR<WorldImpl> w1=shared_from_this();
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			          boost::bind(&WorldImpl::OnUpdate, w1, _1));
	this->rr_path=rr_path;
}

std::string WorldImpl::GetRRPath()
{
	return rr_path;
}



std::string WorldImpl::get_Name()
{
	return get_world()->GetName();
}
void WorldImpl::set_Name(std::string value)
{
	throw std::runtime_error("Read only property");
}

double WorldImpl::get_SimTime()
{
	return get_world()->GetSimTime().Double();
}
void WorldImpl::set_SimTime(double value)
{
	throw std::runtime_error("Read only property");
}

double WorldImpl::get_RealTime()
{
	return get_world()->GetRealTime().Double();
}
void WorldImpl::set_RealTime(double value)
{
	throw std::runtime_error("Read only property");
}

double WorldImpl::get_WallTime()
{
	return common::Time::GetWallTime().Double();
}
void WorldImpl::set_WallTime(double value)
{
	throw std::runtime_error("Read only property");
}

double WorldImpl::get_StartTime()
{
	return get_world()->GetStartTime().Double();
}
void WorldImpl::set_StartTime(double value)
{
	throw std::runtime_error("Read only property");
}

RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > WorldImpl::get_ModelNames()
{
	RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > o=RR_MAKE_SHARED<RR::RRList<RR::RRArray<char>  > >();
	physics::Model_V v=get_world()->GetModels();
	for(auto e=v.begin(); e!=v.end(); e++)
	{
		o->list.push_back(RR::stringToRRArray((*e)->GetName()));
	}
	return o;
}
void WorldImpl::set_ModelNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value)
{
	throw std::runtime_error("Read only property");
}

RR_SHARED_PTR<rrgz::Model> WorldImpl::get_Models(std::string ind)
{
	if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
	physics::ModelPtr m=get_world()->GetModel(ind);
	if (!m) throw std::invalid_argument("Unknown index");
	RR_SHARED_PTR<ModelImpl> m_impl=RR_MAKE_SHARED<ModelImpl>(m);
	m_impl->Init(GetRRPath() + ".Models[" + RR::detail::encode_index(ind) + "]");
	return m_impl;
}

RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > WorldImpl::get_LightNames()
{
	RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > o=RR_MAKE_SHARED<RR::RRList<RR::RRArray<char>  > >();
	physics::Light_V v=get_world()->Lights();
	for(auto e=v.begin(); e!=v.end(); e++)
	{
		o->list.push_back(RR::stringToRRArray((*e)->GetName()));
	}
	return o;
}
void WorldImpl::set_LightNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value)
{
	throw std::runtime_error("Read only property");
}

RR_SHARED_PTR<rrgz::Light> WorldImpl::get_Lights(std::string ind)
{
	//if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
	rendering::ScenePtr scene=rendering::get_scene(get_world()->GetName());
	if (!scene) throw std::invalid_argument("Unknown index");
	physics::LightPtr world_light=get_world()->Light(ind);
	if (!world_light) throw std::invalid_argument("Unknown index");
	rendering::LightPtr l=scene->GetLight(world_light->GetScopedName());
	if (!l) throw std::invalid_argument("Unknown index");
	RR_SHARED_PTR<LightImpl> l_impl=RR_MAKE_SHARED<LightImpl>(l);
	l_impl->Init();
	return l_impl;
}

RR_SHARED_PTR<RR::Wire<double > > WorldImpl::get_SimTimeWire()
{
	boost::mutex::scoped_lock lock(this_lock);
	return m_SimTimeWire;
}
void WorldImpl::set_SimTimeWire(RR_SHARED_PTR<RR::Wire<double > > value)
{
	boost::mutex::scoped_lock lock(this_lock);
	if (m_SimTimeWire) throw std::runtime_error("Already set");
	m_SimTimeWire=value;
	m_SimTimeWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<double> >();
	m_SimTimeWire_b->Init(m_SimTimeWire);
}

RR_SHARED_PTR<RR::Wire<double > > WorldImpl::get_RealTimeWire()
{
	boost::mutex::scoped_lock lock(this_lock);
	return m_RealTimeWire;
}
void WorldImpl::set_RealTimeWire(RR_SHARED_PTR<RR::Wire<double > > value)
{
	boost::mutex::scoped_lock lock(this_lock);
	if (m_RealTimeWire) throw std::runtime_error("Already set");
	m_RealTimeWire=value;
	m_RealTimeWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<double> >();
	m_RealTimeWire_b->Init(m_RealTimeWire);
}

void WorldImpl::OnUpdate(RR_WEAK_PTR<WorldImpl> j, const common::UpdateInfo & _info)
{
	RR_SHARED_PTR<WorldImpl> w1=j.lock();
	if (!w1) return;
	w1->OnUpdate1(_info);
}

void WorldImpl::OnUpdate1(const common::UpdateInfo & _info)
{
	RR_SHARED_PTR<RR::WireBroadcaster<double > > simtime_b;
	RR_SHARED_PTR<RR::WireBroadcaster<double > > realtime_b;
	{
		boost::mutex::scoped_lock lock(this_lock);
		simtime_b=m_SimTimeWire_b;
		realtime_b=m_RealTimeWire_b;
	}

	double simtime=get_SimTime();
	double realtime=get_RealTime();

	if (simtime_b) simtime_b->SetOutValue(simtime);
	if (realtime_b) realtime_b->SetOutValue(realtime);
}

physics::WorldPtr WorldImpl::get_world()
{
	physics::WorldPtr w=gz_world.lock();
	if (!w) throw std::runtime_error("World has been released");
	return w;
}


}
