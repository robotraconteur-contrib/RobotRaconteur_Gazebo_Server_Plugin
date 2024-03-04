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

#include "ModelImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	ModelImpl::ModelImpl(physics::ModelPtr m) : EntityImpl(m)
	{
		gz_model=m;
		gz_world=m->GetWorld();
	}

	void ModelImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		EntityImpl::RRServiceObjectInit(context,service_path);

		RR_SHARED_PTR<physics::Model> m=gz_model.lock();
		if (!m) return;
		auto sdf = m->GetSDF();
		if (sdf->HasElement("gazebo_robotraconteur_server_plugin_model"))
		{
			auto sdf_plugin_model = sdf->GetElement("gazebo_robotraconteur_server_plugin_model");
			
						
			if (sdf_plugin_model->HasElement("kinematic_joint_controller"))			
			{
				auto sdf_plugin_kinematic_joint_controller = sdf_plugin_model->GetElement("kinematic_joint_controller");
				create_kinematic_joint_controller();

				// Fill in joints
				std::set<std::string> joint_names;
				physics::Joint_V v=get_model()->GetJoints();
				for(auto e=v.begin(); e!=v.end(); e++)
				{
					joint_names.insert((*e)->GetName());
				}

				RR_SHARED_PTR<KinematicJointControllerImpl> j=kinematic_joint_controller;

				if (sdf_plugin_kinematic_joint_controller->HasElement("joint"))
				{
					auto sdf_joint = sdf_plugin_kinematic_joint_controller->GetElement("joint");
					while(sdf_joint)
					{
						std::string joint_name = sdf_joint->Get<std::string>("name");
						if (joint_names.find(joint_name) == joint_names.end())
						{
							gzerr << "Kinematic joint controller joint not found: " << joint_name << " for model: " << m->GetName() << std::endl;
							sdf_joint = sdf_joint->GetNextElement("joint");
							continue;
						}
						j->add_joint(joint_name);

						if (sdf_joint->HasElement("initial_position"))
						{
							auto initial_position = sdf_joint->GetElement("initial_position")->Get<double>();
							j->_set_joint_target(joint_name, RR::ScalarToRRArray<double>(initial_position));
						}
						else
						{
							j->_set_joint_target(joint_name, RR::ScalarToRRArray<double>(0.0));
						}
						sdf_joint = sdf_joint->GetNextElement("joint");			
					}
				}
			}
		}
		
	}

	RR::RRListPtr<RR::RRArray<char> > ModelImpl::get_child_model_names()
	{
		RR::RRListPtr<RR::RRArray<char> > o(new RR::RRList<RR::RRArray<char> >());
		auto m=get_model();
		unsigned int n=m->GetChildCount();
		for(unsigned int i=0; i<n; i++)
		{
			auto e=m->GetChild(i);
			if( e->HasType(physics::Base::EntityType::MODEL) )
			{
				o->push_back(RR::stringToRRArray(e->GetName()));
			}
		}
		return o;
	}
	
	rrgz::ModelPtr ModelImpl::get_child_models(const std::string& ind)
	{
		if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
		physics::BasePtr e=get_model()->GetChild(ind);
		if (!e) throw std::invalid_argument("Invalid model name");
		auto m=RR_DYNAMIC_POINTER_CAST<physics::Model>(e);
		if (!m) throw std::invalid_argument("Invalid model name");
		RR_SHARED_PTR<ModelImpl> m_impl=RR_MAKE_SHARED<ModelImpl>(m);
		return m_impl;
	}

	RR::RRListPtr<RR::RRArray<char> > ModelImpl::get_link_names()
	{
		RR::RRListPtr<RR::RRArray<char> > o(new RR::RRList<RR::RRArray<char> >());
		physics::Link_V v=get_model()->GetLinks();
		for(auto e=v.begin(); e!=v.end(); e++)
		{
			o->push_back(RR::stringToRRArray((*e)->GetName()));
		}
		return o;
	}
	
	rrgz::LinkPtr ModelImpl::get_links(const std::string& ind)
	{
		if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
		physics::LinkPtr m=get_model()->GetLink(ind);
		if (!m) throw std::invalid_argument("Unknown index");
		RR_SHARED_PTR<LinkImpl> m_impl=RR_MAKE_SHARED<LinkImpl>(m);
		return m_impl;
	}

	RR::RRListPtr<RR::RRArray<char> > ModelImpl::get_joint_names()
	{
		RR::RRListPtr<RR::RRArray<char> > o(new RR::RRList<RR::RRArray<char> >());
		physics::Joint_V v=get_model()->GetJoints();
		for(auto e=v.begin(); e!=v.end(); e++)
		{
			o->push_back(RR::stringToRRArray((*e)->GetName()));
		}
		return o;
	}
	
	rrgz::JointPtr ModelImpl::get_joints(const std::string& ind)
	{
		if (ind.find(':')!=std::string::npos) throw std::invalid_argument("Do not use scoped names for index");
		physics::JointPtr m=get_model()->GetJoint(ind);
		if (!m) throw std::invalid_argument("Unknown index");
		RR_SHARED_PTR<JointImpl> m_impl=RR_MAKE_SHARED<JointImpl>(m);
		return m_impl;
	}

	physics::ModelPtr ModelImpl::get_model()
	{
		physics::ModelPtr m=gz_model.lock();
		if (!m) throw std::runtime_error("Model has been released");
		return m;
	}

	void ModelImpl::create_joint_controller()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (joint_controller || kinematic_joint_controller) throw std::runtime_error("Joint controller already active");

		RR_SHARED_PTR<JointControllerImpl> j=RR_MAKE_SHARED<JointControllerImpl>(RR_DYNAMIC_POINTER_CAST<ModelImpl>(shared_from_this()), get_model());

		joint_controller=j;		
	}
	void ModelImpl::destroy_joint_controller()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (!joint_controller) return;
		RR::ServerContext::GetCurrentServerContext()->ReleaseServicePath(joint_controller->RRPath());

		joint_controller.reset();
	}

	rrgz::JointControllerPtr ModelImpl::get_joint_controller()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (!joint_controller) throw std::runtime_error("Joint controller not active");
		return joint_controller;
	}

	void ModelImpl::create_kinematic_joint_controller()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (joint_controller || kinematic_joint_controller) throw std::runtime_error("Joint controller already active");

		RR_SHARED_PTR<KinematicJointControllerImpl> j=RR_MAKE_SHARED<KinematicJointControllerImpl>(RR_DYNAMIC_POINTER_CAST<ModelImpl>(shared_from_this()), get_model());

		kinematic_joint_controller=j;

		RR::ServerContextPtr rr_context1 = rr_context.lock();
		if (rr_context1)
		{
			std::string controller_rr_path = this->rr_path + ".kinematic_joint_controller";
			auto node = rr_context1->GetNode();			
			node->GetThreadPool()->Post([rr_context1, controller_rr_path]
			{
				rr_context1->GetObjectSkel(controller_rr_path);
			});
		}
	}
	void ModelImpl::destroy_kinematic_joint_controller()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (!kinematic_joint_controller) return;
		RR::ServerContext::GetCurrentServerContext()->ReleaseServicePath(kinematic_joint_controller->RRPath());

		kinematic_joint_controller.reset();
	}

	rrgz::JointControllerPtr ModelImpl::get_kinematic_joint_controller()
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (!kinematic_joint_controller) throw std::runtime_error("Joint controller not active");
		return kinematic_joint_controller;
	}

}
