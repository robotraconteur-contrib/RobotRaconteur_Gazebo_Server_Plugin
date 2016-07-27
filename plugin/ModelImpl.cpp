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

#include "ModelImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	ModelImpl::ModelImpl(physics::ModelPtr m)
	{
		gz_model=m;
		gz_world=m->GetWorld();
	}

	RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > ModelImpl::get_ChildModelNames()
	{
		RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > o=RR_MAKE_SHARED<RR::RRList<RR::RRArray<char>  > >();
		auto m=get_model();
		unsigned int n=m->GetChildCount();
		for(unsigned int i=0; i<n; i++)
		{
			auto e=m->GetChild(i);
			if( e->HasType(physics::Base::EntityType::MODEL) )
			{
				o->list.push_back(RR::stringToRRArray(e->GetName()));
			}
		}
		return o;
	}
	void ModelImpl::set_ChildModelNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<rrgz::Model > ModelImpl::get_ChildModels(std::string ind)
	{
		if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
		physics::BasePtr e=get_model()->GetChild(ind);
		if (!e) throw std::invalid_argument("Invalid model name");
		auto m=RR_DYNAMIC_POINTER_CAST<physics::Model>(e);
		if (!m) throw std::invalid_argument("Invalid model name");
		RR_SHARED_PTR<ModelImpl> m_impl=RR_MAKE_SHARED<ModelImpl>(m);
		m_impl->Init(GetRRPath() + ".ChildModels[" + RR::detail::encode_index(ind) + "]");
		return m_impl;
	}

	RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > ModelImpl::get_LinkNames()
	{
		RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > o=RR_MAKE_SHARED<RR::RRList<RR::RRArray<char>  > >();
		physics::Link_V v=get_model()->GetLinks();
		for(auto e=v.begin(); e!=v.end(); e++)
		{
			o->list.push_back(RR::stringToRRArray((*e)->GetName()));
		}
		return o;
	}
	void ModelImpl::set_LinkNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<rrgz::Link > ModelImpl::get_Links(std::string ind)
	{
		if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
		physics::LinkPtr m=get_model()->GetLink(ind);
		if (!m) throw std::invalid_argument("Unknown index");
		RR_SHARED_PTR<LinkImpl> m_impl=RR_MAKE_SHARED<LinkImpl>(m);
		m_impl->Init(GetRRPath() + ".Links[" + RR::detail::encode_index(ind) + "]");
		return m_impl;
	}

	RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > ModelImpl::get_JointNames()
	{
		RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > o=RR_MAKE_SHARED<RR::RRList<RR::RRArray<char>  > >();
		physics::Joint_V v=get_model()->GetJoints();
		for(auto e=v.begin(); e!=v.end(); e++)
		{
			o->list.push_back(RR::stringToRRArray((*e)->GetName()));
		}
		return o;
	}

	void ModelImpl::set_JointNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<rrgz::Joint > ModelImpl::get_Joints(std::string ind)
	{
		if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
		physics::JointPtr m=get_model()->GetJoint(ind);
		if (!m) throw std::invalid_argument("Unknown index");
		RR_SHARED_PTR<JointImpl> m_impl=RR_MAKE_SHARED<JointImpl>(m);
		m_impl->Init();
		return m_impl;
	}

	physics::ModelPtr ModelImpl::get_model()
	{
		physics::ModelPtr m=gz_model.lock();
		if (!m) throw std::runtime_error("Model has been released");
		return m;
	}

	physics::EntityPtr ModelImpl::get_entity()
	{
		return boost::dynamic_pointer_cast<physics::Entity>(get_model());
	}

	void ModelImpl::CreateJointController()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (joint_controller) throw std::runtime_error("Joint controller already active");

		RR_SHARED_PTR<JointControllerImpl> j=RR_MAKE_SHARED<JointControllerImpl>(RR_DYNAMIC_POINTER_CAST<ModelImpl>(shared_from_this()), get_model());

		joint_controller=j;
		j->Init();
	}
	void ModelImpl::DestroyJointController()
	{
		boost::mutex::scoped_lock lock(this_lock);
		std::string spath=RR::ServerContext::GetCurrentServicePath() + ".JointController";
		RR::ServerContext::GetCurrentServerContext()->ReleaseServicePath(spath);

		joint_controller.reset();
	}

	RR_SHARED_PTR<rrgz::JointController > ModelImpl::get_JointController()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (!joint_controller) throw std::runtime_error("Joint controller not active");
		return joint_controller;
	}

}
