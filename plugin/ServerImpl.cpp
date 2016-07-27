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

#include "ServerImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	void ServerImpl::Init()
	{
		worldCreatedConnection=event::Events::ConnectWorldCreated(
				boost::bind(&ServerImpl::OnWorldCreated,shared_from_this(),_1));

	}

	void ServerImpl::OnWorldCreated(std::string name)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if(rr_worlds.count(name)==0)
		{
			physics::WorldPtr w=physics::get_world(name);
			if (!w) return;

			RR_SHARED_PTR<WorldImpl> rr_w=RR_MAKE_SHARED<WorldImpl>(w);
			rr_w->Init("GazeboServer.Worlds[" + RR::detail::encode_index(w->GetName()) +  "]");
			rr_worlds.insert(std::make_pair(w->GetName(),rr_w));
		}
	}

	RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > ServerImpl::get_WorldNames()
	{
		boost::mutex::scoped_lock lock(this_lock);
		RR_SHARED_PTR<RR::RRList<RR::RRArray<char> > > o=RR_MAKE_SHARED<RR::RRList<RR::RRArray<char> > >();
		for (auto e=rr_worlds.begin(); e!=rr_worlds.end(); e++)
		{
			o->list.push_back(RR::stringToRRArray(e->first));
		}
		return o;

	}
	void ServerImpl::set_WorldNames(RR_SHARED_PTR<RobotRaconteur::RRList<RobotRaconteur::RRArray<char>  > > value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<rrgz::World > ServerImpl::get_Worlds(std::string ind)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (rr_worlds.count(ind)==0) throw std::invalid_argument("Invalid world");
		return rr_worlds.at(ind);
	}

	RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > ServerImpl::get_SensorNames()
	{
		sensors::Sensor_V s=sensors::SensorManager::Instance()->GetSensors();
		auto o=RR_MAKE_SHARED<RR::RRList<RR::RRArray<char>  > >();

		for (auto e=s.begin(); e!=s.end(); e++)
		{
			o->list.push_back(RR::stringToRRArray((*e)->ScopedName()));
		}
		return o;
	}
	void ServerImpl::set_SensorNames(RR_SHARED_PTR<RR::RRList<RR::RRArray<char>  > > value)
	{
		throw std::runtime_error("Read only property");
	}


	RR_SHARED_PTR<rrgz::Sensor > ServerImpl::get_Sensors(std::string ind)
	{
		sensors::SensorPtr s=sensors::SensorManager::Instance()->GetSensor(ind);
		if (!s) throw std::invalid_argument("Invalid sensor name");

		sensors::CameraSensorPtr gz_camera=std::dynamic_pointer_cast<sensors::CameraSensor>(s);
		if (gz_camera)
		{
			auto rr_camera=RR_MAKE_SHARED<CameraImpl>(gz_camera);
			rr_camera->Init();
			return rr_camera;
		}

		sensors::MultiCameraSensorPtr gz_multi_camera=std::dynamic_pointer_cast<sensors::MultiCameraSensor>(s);
		if (gz_multi_camera)
		{
			auto rr_camera=RR_MAKE_SHARED<MultiCameraSensorImpl>(gz_multi_camera);
			rr_camera->Init();
			return rr_camera;
		}

		sensors::DepthCameraSensorPtr gz_depth_camera=std::dynamic_pointer_cast<sensors::DepthCameraSensor>(s);
		if (gz_depth_camera)
		{
			auto rr_camera=RR_MAKE_SHARED<DepthCameraSensorImpl>(gz_depth_camera);
			rr_camera->Init();
			return rr_camera;
		}

		sensors::RaySensorPtr gz_ray_sensor=std::dynamic_pointer_cast<sensors::RaySensor>(s);
		if (gz_ray_sensor)
		{
			auto rr_ray=RR_MAKE_SHARED<RaySensorImpl>(gz_ray_sensor);
			rr_ray->Init();
			return rr_ray;
		}

		sensors::GpuRaySensorPtr gz_gpu_ray_sensor=std::dynamic_pointer_cast<sensors::GpuRaySensor>(s);
		if (gz_gpu_ray_sensor)
		{
			auto rr_ray=RR_MAKE_SHARED<GpuRaySensorImpl>(gz_gpu_ray_sensor);
			rr_ray->Init();
			return rr_ray;
		}

		sensors::ContactSensorPtr gz_contact_sensor=std::dynamic_pointer_cast<sensors::ContactSensor>(s);
		if (gz_contact_sensor)
		{
			auto rr_contact=RR_MAKE_SHARED<ContactSensorImpl>(gz_contact_sensor);
			rr_contact->Init();
			return rr_contact;
		}

		sensors::AltimeterSensorPtr gz_altimeter_sensor=std::dynamic_pointer_cast<sensors::AltimeterSensor>(s);
		if (gz_altimeter_sensor)
		{
			auto rr_altimeter=RR_MAKE_SHARED<AltimeterSensorImpl>(gz_altimeter_sensor);
			rr_altimeter->Init();
			return rr_altimeter;
		}

		sensors::SonarSensorPtr gz_sonar_sensor=std::dynamic_pointer_cast<sensors::SonarSensor>(s);
		if (gz_sonar_sensor)
		{
			auto rr_sonar=RR_MAKE_SHARED<SonarSensorImpl>(gz_sonar_sensor);
			rr_sonar->Init();
			return rr_sonar;
		}

		sensors::MagnetometerSensorPtr gz_magnetometer_sensor=std::dynamic_pointer_cast<sensors::MagnetometerSensor>(s);
		if (gz_magnetometer_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<MagnetometerSensorImpl>(gz_magnetometer_sensor);
			rr_m->Init();
			return rr_m;
		}

		sensors::ForceTorqueSensorPtr gz_ft_sensor=std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(s);
		if (gz_ft_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<ForceTorqueSensorImpl>(gz_ft_sensor);
			rr_m->Init();
			return rr_m;
		}

		sensors::GpsSensorPtr gz_gps_sensor=std::dynamic_pointer_cast<sensors::GpsSensor>(s);
		if (gz_gps_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<GpsSensorImpl>(gz_gps_sensor);
			rr_m->Init();
			return rr_m;
		}

		sensors::ImuSensorPtr gz_imu_sensor=std::dynamic_pointer_cast<sensors::ImuSensor>(s);
		if (gz_imu_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<ImuSensorImpl>(gz_imu_sensor);
			rr_m->Init();
			return rr_m;
		}

		auto rr_s=RR_MAKE_SHARED<SensorImpl>(s);
		return rr_s;
	}


}
