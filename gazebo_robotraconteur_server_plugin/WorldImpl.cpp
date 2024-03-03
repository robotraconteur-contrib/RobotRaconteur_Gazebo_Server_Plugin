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

void WorldImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
{
	auto context1 = context.lock();
	if (!context1) return;
	rr_node=context1->GetNode();
	RR_WEAK_PTR<WorldImpl> w1=shared_from_this();
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			          boost::bind(&WorldImpl::OnUpdate, w1, _1));
	this->rr_path=service_path;

	RR_WEAK_PTR<WorldImpl> weak_this = shared_from_this();
	rrvar_sim_time->GetWire()->SetPeekInValueCallback(
		[weak_this](const uint32_t&)
		{
			auto this_ = weak_this.lock();
			if (!this_) throw RR::InvalidOperationException("Gazebo world object not found");
			return gz_time_to_rr_duration(this_->get_world()->SimTime());
		}
	);

	rrvar_time->GetWire()->SetPeekInValueCallback(
		[weak_this](const uint32_t&)
		{
			auto this_ = weak_this.lock();
			if (!this_) throw RR::InvalidOperationException("Gazebo world object not found");
			physics::WorldPtr w = this_->get_world();			
			
			return gz_to_rr_worldtimes(w);
		}
	);

	gz_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	gz_node->Init(get_world()->Name());
	gz_request_pub = gz_node->Advertise<gazebo::msgs::Request>("~/request");
	gz_factory_pub = gz_node->Advertise<gazebo::msgs::Factory>("~/factory");

	// Touch all models to activate RR skels
	
	physics::WorldPtr w=gz_world.lock();	
	if (context1 && w)
	{
		// Iterate over models
		physics::Model_V v=w->Models();
		for(auto e=v.begin(); e!=v.end(); e++)
		{
			std::string model_name = (*e)->GetName();
			context1->GetNode()->GetThreadPool()->Post([context1, model_name, service_path]()
			{
				try
				{
					std::string model_path = service_path + ".models[" + RR::detail::encode_index(model_name) + "]";
					context1->GetObjectSkel(model_path);
					gzmsg << "Initialized model " << model_name << " with RR path " << model_path << std::endl;
				}
				catch (std::exception& e)
				{
					gzerr << "Error initializing model with RR: " << e.what() << std::endl;
				}
			});
		}			
	}
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

void WorldImpl::OnUpdate(RR_WEAK_PTR<WorldImpl> j, const common::UpdateInfo & _info)
{
	RR_SHARED_PTR<WorldImpl> w1=j.lock();
	if (!w1) return;
	w1->OnUpdate1(_info);
}

void WorldImpl::OnUpdate1(const common::UpdateInfo & _info)
{
	auto w = gz_world.lock();
	if (!w) return;	
	
	rrvar_sim_time->SetOutValue(gz_time_to_rr_duration(w->SimTime()));
	rrvar_time->SetOutValue(gz_to_rr_worldtimes(w));

	for (auto e = insert_ops.begin(); e!=insert_ops.end();)
	{

		if (w->ModelByName(e->model_name))
		{
			auto rr_handler = e->handler;
			RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [rr_handler](){
				rr_handler(nullptr);
			});
			e=insert_ops.erase(e);
		}
		else if (common::Time::GetWallTime() > e->insert_time + common::Time(10,0))
		{
			auto model_name = e->model_name;
			auto rr_handler = e->handler;
			RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [rr_handler,model_name](){
				rr_handler(RR_MAKE_SHARED<RR::RequestTimeoutException>("Model insert for " + model_name + " timed out"));
			});
			e=insert_ops.erase(e);			
		}
		else
		{
			e++;
		}
	}

}

physics::WorldPtr WorldImpl::get_world()
{
	physics::WorldPtr w=gz_world.lock();
	if (!w) throw std::runtime_error("World has been released");
	return w;
}

void WorldImpl::insert_model(const std::string& model_sdf, const std::string& model_name, const geometry::Pose& model_pose)
{
	physics::WorldPtr w=gz_world.lock();
	if (!w) throw RR::InvalidOperationException("World has been released");

	if(w->ModelByName(model_name))
	{
		throw RR::InvalidOperationException("Model name " + model_name + " already in use");
	}

	auto model_quat = model_pose.s.orientation;
	if (model_quat.s.w == 0 && model_quat.s.x == 0 && model_quat.s.y == 0 && model_quat.s.z == 0)
	{
		model_quat.s.w = 1.0;
	}
	auto model_pos = model_pose.s.position;

	ignition::math::Pose3d p(model_pos.s.x, model_pos.s.y, model_pos.s.z, model_quat.s.w, model_quat.s.x, model_quat.s.y, model_quat.s.z);

	sdf::SDF modelSDF;
	/*if (!sdf::readString(model_sdf, modelSDF.Root()))
	{
		throw RR::InvalidArgumentException("Could not parse specified SDF model");
	}*/

	modelSDF.SetFromString(model_sdf);

	sdf::ElementPtr model = modelSDF.Root()->GetElement("model");
	if (!model)
	{
		throw RR::InvalidArgumentException("Model not found in specified SDF");
	}

	model->GetAttribute("name")->SetFromString(model_name);
	auto pose_elem = model->GetElement("pose");
	if (pose_elem)
	{
		pose_elem->Set(p);
	}
	else
	{
		pose_elem = model->AddElement("pose");
		pose_elem->Set(p);
	}
	//w->InsertModelSDF(modelSDF);
  gazebo::msgs::Factory msg;
  gazebo::msgs::Init(msg, "spawn_model");
  msg.set_sdf( modelSDF.ToString() );

  gz_factory_pub->Publish(msg);
}

void WorldImpl::remove_model(const std::string& model_name)
{
	physics::WorldPtr w=gz_world.lock();
	if (!w) throw RR::InvalidOperationException("World has been released");

	auto model = w->ModelByName(model_name);
	if(!model)
	{
		throw RR::InvalidArgumentException("Model name " + model_name + " not found");
	}

	//w->RemoveModel(model_name);
	//model->Fini();

	gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest("entity_delete",model->GetScopedName());
    gz_request_pub->Publish(*msg,false);
	delete msg;
	
}

void WorldImpl::async_get_name(boost::function<void (const std::string&,const RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>&) > rr_handler, int32_t rr_timeout)
{
	std::string name = get_name();
	RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [rr_handler,name] { rr_handler(name,nullptr); });
}

void WorldImpl::async_get_model_names(boost::function<void (const RR_INTRUSIVE_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > >&,const RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>&) > rr_handler, int32_t rr_timeout)
{
	auto model_names = get_model_names();
	RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [rr_handler,model_names] { rr_handler(model_names,nullptr); });
}

void WorldImpl::async_get_light_names(boost::function<void (const RR_INTRUSIVE_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > >&,const RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>&) > rr_handler, int32_t rr_timeout)
{
	auto light_names = get_light_names();
	RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [rr_handler,light_names] { rr_handler(light_names,nullptr); });
}

void WorldImpl::async_insert_model(const std::string& model_sdf, const std::string& model_name, const com::robotraconteur::geometry::Pose& model_pose,boost::function<void (const RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>&) > rr_handler, int32_t rr_timeout)
{
	insert_model(model_sdf, model_name, model_pose);
	//RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [rr_handler] { rr_handler(nullptr); });
	WorldImpl_insert_op op;
	op.handler = rr_handler;
	op.model_name = model_name;
	op.insert_time = common::Time::GetWallTime();

	insert_ops.push_back(op);

}

void WorldImpl::async_remove_model(const std::string& model_name,boost::function<void (const RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>&) > rr_handler, int32_t rr_timeout)
{
	remove_model(model_name);
	RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [rr_handler] { rr_handler(nullptr); });
}

void WorldImpl::async_get_models(const std::string& ind, boost::function<void(RR_SHARED_PTR<rrgz::Model>,const RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>&)> handler, int32_t timeout)
{
	auto m = get_models(ind);
	RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [handler,m] { handler(m,nullptr); });
}

void WorldImpl::async_get_lights(const std::string& ind, boost::function<void(RR_SHARED_PTR<rrgz::Light>,const RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>&)> handler, int32_t timeout)
{
	auto m = get_lights(ind);
	RR::RobotRaconteurNode::TryPostToThreadPool(rr_node, [handler,m] { handler(m,nullptr); });
}


}
