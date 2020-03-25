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



std::string WorldImpl::get_name()
{
	return get_world()->Name();
}

RR::RRListPtr<RR::RRArray<char> > WorldImpl::get_model_names()
{
	RR::RRListPtr<RR::RRArray<char> > o( new RR::RRList<RR::RRArray<char> >());
	physics::Model_V v=get_world()->Models();
	for(auto e=v.begin(); e!=v.end(); e++)
	{
		o->push_back(RR::stringToRRArray((*e)->GetName()));
	}
	return o;
}

rrgz::ModelPtr WorldImpl::get_models(const std::string& ind)
{
	if (ind.find(':')!=std::string::npos) throw RR::InvalidArgumentException("Do not use scoped names for index");
	physics::ModelPtr m=get_world()->ModelByName(ind);
	if (!m) throw RR::InvalidArgumentException("Unknown index");
	RR_SHARED_PTR<ModelImpl> m_impl=RR_MAKE_SHARED<ModelImpl>(m);
	return m_impl;
}

RR::RRListPtr<RR::RRArray<char> > WorldImpl::get_light_names()
{
	RR::RRListPtr<RR::RRArray<char> > o(new RR::RRList<RR::RRArray<char> >());
	physics::Light_V v=get_world()->Lights();
	for(auto e=v.begin(); e!=v.end(); e++)
	{
		o->push_back(RR::stringToRRArray((*e)->GetName()));
	}
	return o;
}

rrgz::LightPtr WorldImpl::get_lights(const std::string& ind)
{
	//if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
	rendering::ScenePtr scene=rendering::get_scene(get_world()->Name());
	if (!scene) throw RR::InvalidArgumentException("Unknown index");
	physics::LightPtr world_light=get_world()->LightByName(ind);
	if (!world_light) throw RR::InvalidArgumentException("Unknown index");
	rendering::LightPtr l=scene->GetLight(world_light->GetScopedName());
	if (!l) throw RR::InvalidArgumentException("Unknown index");
	RR_SHARED_PTR<LightImpl> l_impl=RR_MAKE_SHARED<LightImpl>(l);
	return l_impl;
}

static datetime::Duration gz_time_to_rr_duration(const gazebo::common::Time& t)
{
	datetime::Duration o;
	o.clock_info.clock_type = datetime::ClockTypeCode::sim_clock_scaled;
	o.seconds = t.sec;
	o.nanoseconds = t.nsec;
	return o;
}

static datetime::DateTimeUTC gz_time_to_rr_datetimeutc(const gazebo::common::Time& t)
{
	datetime::DateTimeUTC o;
	o.clock_info.clock_type = datetime::ClockTypeCode::sim_clock_scaled;
	o.seconds = t.sec;
	o.nanoseconds = t.nsec;
	return o;
}

static rrgz::WorldTimesPtr gz_to_rr_worldtimes(gazebo::physics::WorldPtr& world)
{
	rrgz::WorldTimesPtr t(new rrgz::WorldTimes());

	t->sim_time = gz_time_to_rr_duration(world->SimTime());
	t->real_time = gz_time_to_rr_duration(world->RealTime());
	t->wall_time = gz_time_to_rr_datetimeutc(common::Time::GetWallTime());
	t->start_time = gz_time_to_rr_datetimeutc(world->StartTime());
	return t;
}

void WorldImpl::set_sim_time(RR::WirePtr<datetime::Duration> value)
{
	rrgz::World_default_impl::set_sim_time(value);
	RR_WEAK_PTR<WorldImpl> weak_this = shared_from_this();
	rrvar_sim_time->GetWire()->SetPeekInValueCallback(
		[weak_this](const uint32_t&)
		{
			auto this_ = weak_this.lock();
			if (!this_) throw RR::InvalidOperationException("Gazebo world object not found");
			return gz_time_to_rr_duration(this_->get_world()->SimTime());
		}
	);
}

void WorldImpl::set_time(RR::WirePtr<rrgz::WorldTimesPtr> value)
{
	rrgz::World_default_impl::set_time(value);
	RR_WEAK_PTR<WorldImpl> weak_this = shared_from_this();
	rrvar_time->GetWire()->SetPeekInValueCallback(
		[weak_this](const uint32_t&)
		{
			auto this_ = weak_this.lock();
			if (!this_) throw RR::InvalidOperationException("Gazebo world object not found");
			physics::WorldPtr w = this_->get_world();			
			
			return gz_to_rr_worldtimes(w);
		}
	);
}

void WorldImpl::OnUpdate(RR_WEAK_PTR<WorldImpl> j, const common::UpdateInfo & _info)
{
	RR_SHARED_PTR<WorldImpl> w1=j.lock();
	if (!w1) return;
	w1->OnUpdate1(_info);
}

void WorldImpl::OnUpdate1(const common::UpdateInfo & _info)
{
	RR_SHARED_PTR<RR::WireBroadcaster<rrgz::WorldTimesPtr > > time_b;
	RR_SHARED_PTR<RR::WireBroadcaster<datetime::Duration > > simtime_b;
	
	{
		boost::mutex::scoped_lock lock(this_lock);
		time_b = rrvar_time;
		simtime_b = rrvar_sim_time;
	}
	
	physics::WorldPtr w = get_world();

	if (simtime_b) simtime_b->SetOutValue(gz_time_to_rr_duration(w->SimTime()));
	if (time_b) time_b->SetOutValue(gz_to_rr_worldtimes(w));
}

physics::WorldPtr WorldImpl::get_world()
{
	physics::WorldPtr w=gz_world.lock();
	if (!w) throw std::runtime_error("World has been released");
	return w;
}


}
