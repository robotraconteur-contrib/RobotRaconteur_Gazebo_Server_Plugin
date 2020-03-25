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

#include "robotraconteur_gazebo_server_plugin.h"
#include <memory>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace RobotRaconteurGazeboServerPlugin
{
	RobotRaconteurGazeboServerPlugin::RobotRaconteurGazeboServerPlugin() : SystemPlugin()
	{
		tcp_port=-1;
		tcp_port_sharer=false;
		tcp_node_announce=false;
		tcp_load_tls=false;
	}

	void RobotRaconteurGazeboServerPlugin::Load(int _argc, char ** _argv)
	{

		try
		{
			bool use_nodename=false;
			bool use_nodeid=false;

			po::options_description desc("Allowed options");
			desc.add_options()
					("robotraconteur-server-nodeid", po::value<std::string>(), "set nodeid of plugin node")
					("robotraconteur-server-nodename", po::value<std::string>(), "set nodename of plugin node")
					("robotraconteur-server-tcp-port", po::value<std::string>(), "set port for TcpTransport of plugin node or \"sharer\" to use port sharer")
					("robotraconteur-server-tcp-announce", "enable node announce for TcpTransport of plugin node")
					("robotraconteur-server-tcp-loadtls", "load TLS certificate for TcpTransport of plugin node")
					("robotraconteur-server-password-file", po::value<std::string>(), "password file of plugin node");

			po::variables_map vm;
			po::store(po::command_line_parser(_argc, _argv).options(desc).allow_unregistered().run(), vm);
			po::notify(vm);


			if (vm.count("robotraconteur-server-nodeid"))
			{
				std::string nodeid_str=vm["robotraconteur-server-nodeid"].as<std::string>();
				nodeid=RR::NodeID(nodeid_str);
				use_nodeid=true;
			}

			if(vm.count("robotraconteur-server-nodename"))
			{
				if (use_nodeid==true)
				{
					throw std::invalid_argument("Only nodename or nodeid may be specified, but not both");
				}

				nodename=vm["robotraconteur-server-nodename"].as<std::string>();
				use_nodename=true;
			}

			if (vm.count("robotraconteur-server-tcp-port"))
			{
				std::string tcpport_s=vm["robotraconteur-server-tcp-port"].as<std::string>();
				if (tcpport_s=="sharer")
				{
					tcp_port_sharer=true;
				}
				else
				{
					try
					{
						tcp_port=boost::lexical_cast<int>(tcpport_s);
					}
					catch (boost::bad_lexical_cast&)
					{
						gzerr << "Robot Raconteur Server Plugin: Invalid TCP port" << std::endl;
					}
				}

				if (tcp_port > 0 || tcp_port_sharer)
				{
					if (vm.count("robotraconteur-server-tcp-announce"))
					{
						tcp_node_announce=true;
					}

					if (vm.count("robotraconteur-server-tcp-loadtls"))
					{
						tcp_load_tls=true;
					}
				}

			}

			if (vm.count("robotraconteur-server-password-file"))
			{
				password_file=vm["robotraconteur-server-password-file"].as<std::string>();
			}

			if (use_nodeid==false && use_nodename==false)
			{
				nodename="experimental.gazebo.GazeboServer";
			}

			//tcp_port=11346;

		}
		catch (std::exception& e)
		{
			gzerr << "Exception occurred in Robot Raconteur Server Plugin: " << e.what() << "\n";
			return;
		}
	}

	void RobotRaconteurGazeboServerPlugin::Init()
	{
		rr_start();
	}

	void RobotRaconteurGazeboServerPlugin::Reset()
	{

	}

	RobotRaconteurGazeboServerPlugin::~RobotRaconteurGazeboServerPlugin()
	{
		rr_stop();
	}

	void RobotRaconteurGazeboServerPlugin::rr_start()
	{
		try
		{
			boost::mutex::scoped_lock lock(this_lock);

			rr_node=RR_MAKE_SHARED<RR::RobotRaconteurNode>();
			thread_pool = RR_MAKE_SHARED<RR::IOContextThreadPool>(rr_node,boost::ref(io_context),false);
			rr_node->SetThreadPool(thread_pool);
			rr_node->Init();

			rr_node->RegisterServiceType(RR_MAKE_SHARED<rrgz::experimental__gazeboFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::geometry::com__robotraconteur__geometryFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::image::com__robotraconteur__imageFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::color::com__robotraconteur__colorFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::datetime::com__robotraconteur__datetimeFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::resource::com__robotraconteur__resourceFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::identifier::com__robotraconteur__identifierFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::uuid::com__robotraconteur__uuidFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::laserscan::com__robotraconteur__laserscanFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::pid::com__robotraconteur__pidFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::sensordata::com__robotraconteur__sensordataFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::gps::com__robotraconteur__gpsFactory>());			
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::imu::com__robotraconteur__imuFactory>());		
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::sensor::com__robotraconteur__sensorFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::param::com__robotraconteur__paramFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::datatype::com__robotraconteur__datatypeFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::units::com__robotraconteur__unitsFactory>());
			rr_node->RegisterServiceType(RR_MAKE_SHARED<com::robotraconteur::device::com__robotraconteur__deviceFactory>());

			rr_local_transport=RR_MAKE_SHARED<RR::LocalTransport>(rr_node);


			if (nodeid.IsAnyNode())
			{
				rr_local_transport->StartServerAsNodeName(nodename);
			}
			else
			{
				rr_local_transport->StartServerAsNodeID(nodeid);
			}

			rr_node->RegisterTransport(rr_local_transport);


			if (tcp_port>0 || tcp_port_sharer)
			{
				if (tcp_port_sharer)
				{
					gzmsg << "Robot Raconteur Server Plugin listening on using TCP port sharer " << std::endl;
				}
				else
				{
					gzmsg << "Robot Raconteur Server Plugin listening on TCP port: " << tcp_port << std::endl;
				}
				rr_tcp_transport=RR_MAKE_SHARED<RR::TcpTransport>(rr_node);
				if (tcp_port_sharer)
				{
					rr_tcp_transport->StartServerUsingPortSharer();
				}
				else
				{
					rr_tcp_transport->StartServer(tcp_port);
				}
				if (tcp_node_announce)
				{
					rr_tcp_transport->EnableNodeAnnounce();
				}

				if(tcp_load_tls)
				{
					try
					{
						rr_tcp_transport->LoadTlsNodeCertificate();
					}
					catch (std::exception&)
					{
						gzwarn << "Could not load Robot Raconteur Server Plugin TLS certificate" << std::endl;
					}
				}

				rr_node->RegisterTransport(rr_tcp_transport);
			}
			else
			{
				if (nodename=="")
				{
					gzmsg << "Robot Raconteur Server Plugin listening locally with NodeName: " << nodeid.ToString() << std::endl;
				}
				else
				{
					gzmsg << "Robot Raconteur Server Plugin listening locally with NodeName: " << nodename << std::endl;
				}
			}


			server=RR_MAKE_SHARED<ServerImpl>();
			
			if(password_file=="")
			{
				rr_node->RegisterService("GazeboServer", "experimental.gazebo", server);
			}
			else
			{

				RR_SHARED_PTR<RR::PasswordFileUserAuthenticator> p;
				try
				{
					std::string authdata=ReadFile(password_file);
					p=RR_MAKE_SHARED<RR::PasswordFileUserAuthenticator>(authdata);
				}
				catch (std::exception&)
				{
					throw std::runtime_error("could not load password file");
				}

				std::map<std::string,std::string> policies;
				policies.insert(std::make_pair("requirevaliduser","true"));
				policies.insert(std::make_pair("allowobjectlock","true"));

				RR_SHARED_PTR<RR::ServiceSecurityPolicy> s=RR_MAKE_SHARED<RR::ServiceSecurityPolicy>(p,policies);

			}

			on_world_update_begin_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RobotRaconteurGazeboServerPlugin::on_world_update_begin,this,_1));
		}
		catch (std::exception& e)
		{
			gzerr << "Could not start Robot Raconteur Server Plugin: " << std::string(e.what()) << std::endl;
		}

	}

	void RobotRaconteurGazeboServerPlugin::rr_stop()
	{
		boost::mutex::scoped_lock lock(this_lock);

		if (rr_node)
		{
			//rr_node->Shutdown();
			on_world_update_begin_connection.reset();
			rr_node.reset();
		}

	}

	std::string RobotRaconteurGazeboServerPlugin::ReadFile(const std::string& fname)
	{
		std::ifstream file(fname.c_str(),std::ios_base::in);

		if (!file.is_open()) throw std::runtime_error("File not found");

		char bom1, bom2, bom3;
		file >> bom1 >> bom2 >> bom3;
		if (!(bom1==-17 && bom2==-69 && bom3==-65))
		{
			file.seekg(0, std::ifstream::beg);
		}

		std::stringstream buffer;
		buffer << file.rdbuf();
		file.close();

		return buffer.str();

	}

	void RobotRaconteurGazeboServerPlugin::on_world_update_begin(const common::UpdateInfo& world_info)
	{
		io_context.poll();
	}

  GZ_REGISTER_SYSTEM_PLUGIN(RobotRaconteurGazeboServerPlugin)
}
