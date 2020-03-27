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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <RobotRaconteur.h>
#include "experimental__gazebo.h"
#include "experimental__gazebo_stubskel.h"

#pragma once

using namespace gazebo;
namespace RR=RobotRaconteur;
namespace rrgz=experimental::gazebo;
namespace datetime = com::robotraconteur::datetime;
namespace geometry = com::robotraconteur::geometry;

namespace RobotRaconteurGazeboServerPlugin
{

  class WorldImpl_insert_op
  {
  public:
    std::string model_name;
	common::Time insert_time;
	boost::function<void (RR::RobotRaconteurExceptionPtr) > handler;
  };


  class WorldImpl : public virtual rrgz::World_default_impl, public virtual rrgz::async_World, public virtual RR::IRRServiceObject, public RR_ENABLE_SHARED_FROM_THIS<WorldImpl>
  {
  public:
	  WorldImpl(physics::WorldPtr w);

	  virtual void RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path) override;

	  virtual std::string get_name() override;	  

	  static void OnUpdate(RR_WEAK_PTR<WorldImpl> j, const common::UpdateInfo & _info);

	  virtual RR::RRListPtr<RR::RRArray<char> > get_model_names() override;	  

	  virtual RR::RRListPtr<RR::RRArray<char> > get_light_names() override;
	  	  
	  virtual rrgz::ModelPtr get_models(const std::string& ind) override;
	  virtual rrgz::LightPtr get_lights(const std::string& ind) override;

	  physics::WorldPtr get_world();

	  std::string GetRRPath();

	  virtual void insert_model(const std::string& model_sdf, const std::string& model_name, const geometry::Pose& model_pose);

	  virtual void remove_model(const std::string& model_name);

	  // Async versions of functions to allow for delayed returns

	  virtual void async_get_name(boost::function<void (const std::string&,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

	  virtual void async_get_model_names(boost::function<void (RR_INTRUSIVE_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

	  virtual void async_get_light_names(boost::function<void (RR_INTRUSIVE_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

	  virtual void async_insert_model(const std::string& model_sdf, const std::string& model_name, const com::robotraconteur::geometry::Pose& model_pose,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);


	  virtual void async_remove_model(const std::string& model_name,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);


	  virtual void async_get_models(const std::string& ind, boost::function<void(RR_SHARED_PTR<rrgz::Model>,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)> handler, int32_t timeout=RR_TIMEOUT_INFINITE);

	  virtual void async_get_lights(const std::string& ind, boost::function<void(RR_SHARED_PTR<rrgz::Light>,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)> handler, int32_t timeout=RR_TIMEOUT_INFINITE);



  protected:
	  boost::weak_ptr<physics::World> gz_world;
	  	  
	  virtual void OnUpdate1(const common::UpdateInfo & _info);

	  event::ConnectionPtr updateConnection;

	  std::string rr_path;

	  RR_WEAK_PTR<RR::RobotRaconteurNode> rr_node;

	  std::list<WorldImpl_insert_op> insert_ops;

	  transport::NodePtr gz_node;
	  transport::PublisherPtr gz_request_pub;
	  transport::PublisherPtr gz_factory_pub;




  };

}
