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

#include "ServerImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

#include "RobotRaconteurCompanion/Util/IdentifierUtil.h"

namespace device = com::robotraconteur::device;
namespace RRU = RobotRaconteur::Companion::Util;

namespace RobotRaconteurGazeboServerPlugin
{
	void ServerImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
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
			rr_worlds.insert(std::make_pair(w->Name(),rr_w));
		}
	}

	RR::RRListPtr<RR::RRArray<char> > ServerImpl::get_world_names()
	{
		boost::mutex::scoped_lock lock(this_lock);
		RR::RRListPtr<RR::RRArray<char> > o=RR::AllocateEmptyRRList<RR::RRArray<char> >();
		for (auto e=rr_worlds.begin(); e!=rr_worlds.end(); e++)
		{
			o->push_back(RR::stringToRRArray(e->first));
		}
		return o;

	}
	
	rrgz::WorldPtr ServerImpl::get_worlds(const std::string& ind)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (rr_worlds.count(ind)==0) throw std::invalid_argument("Invalid world");
		return rr_worlds.at(ind);
	}

	RR::RRListPtr<RR::RRArray<char> > ServerImpl::get_sensor_names()
	{
		sensors::Sensor_V s=sensors::SensorManager::Instance()->GetSensors();
		auto o=RR::AllocateEmptyRRList<RR::RRArray<char> >();

		for (auto e=s.begin(); e!=s.end(); e++)
		{
			o->push_back(RR::stringToRRArray((*e)->ScopedName()));
		}
		return o;
	}
	
	rrgz::SensorPtr ServerImpl::get_sensors(const std::string& ind)
	{
		sensors::SensorPtr s=sensors::SensorManager::Instance()->GetSensor(ind);
		if (!s) throw std::invalid_argument("Invalid sensor name");

		sensors::DepthCameraSensorPtr gz_depth_camera=std::dynamic_pointer_cast<sensors::DepthCameraSensor>(s);
		if (gz_depth_camera)
		{
			auto rr_camera=RR_MAKE_SHARED<DepthCameraSensorImpl>(gz_depth_camera);
			return rr_camera;
		}

		sensors::CameraSensorPtr gz_camera=std::dynamic_pointer_cast<sensors::CameraSensor>(s);
		if (gz_camera)
		{
			auto rr_camera=RR_MAKE_SHARED<CameraImpl>(gz_camera);
			return rr_camera;
		}

		sensors::MultiCameraSensorPtr gz_multi_camera=std::dynamic_pointer_cast<sensors::MultiCameraSensor>(s);
		if (gz_multi_camera)
		{
			auto rr_camera=RR_MAKE_SHARED<MultiCameraSensorImpl>(gz_multi_camera);
			return rr_camera;
		}
		
		sensors::RaySensorPtr gz_ray_sensor=std::dynamic_pointer_cast<sensors::RaySensor>(s);
		if (gz_ray_sensor)
		{
			auto rr_ray=RR_MAKE_SHARED<RaySensorImpl>(gz_ray_sensor);
			return rr_ray;
		}

		sensors::GpuRaySensorPtr gz_gpu_ray_sensor=std::dynamic_pointer_cast<sensors::GpuRaySensor>(s);
		if (gz_gpu_ray_sensor)
		{
			auto rr_ray=RR_MAKE_SHARED<GpuRaySensorImpl>(gz_gpu_ray_sensor);
			return rr_ray;
		}

		sensors::ContactSensorPtr gz_contact_sensor=std::dynamic_pointer_cast<sensors::ContactSensor>(s);
		if (gz_contact_sensor)
		{
			auto rr_contact=RR_MAKE_SHARED<ContactSensorImpl>(gz_contact_sensor);
			return rr_contact;
		}

		sensors::AltimeterSensorPtr gz_altimeter_sensor=std::dynamic_pointer_cast<sensors::AltimeterSensor>(s);
		if (gz_altimeter_sensor)
		{
			auto rr_altimeter=RR_MAKE_SHARED<AltimeterSensorImpl>(gz_altimeter_sensor);
			return rr_altimeter;
		}

		sensors::SonarSensorPtr gz_sonar_sensor=std::dynamic_pointer_cast<sensors::SonarSensor>(s);
		if (gz_sonar_sensor)
		{
			auto rr_sonar=RR_MAKE_SHARED<SonarSensorImpl>(gz_sonar_sensor);
			return rr_sonar;
		}

		sensors::MagnetometerSensorPtr gz_magnetometer_sensor=std::dynamic_pointer_cast<sensors::MagnetometerSensor>(s);
		if (gz_magnetometer_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<MagnetometerSensorImpl>(gz_magnetometer_sensor);
			return rr_m;
		}

		sensors::ForceTorqueSensorPtr gz_ft_sensor=std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(s);
		if (gz_ft_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<ForceTorqueSensorImpl>(gz_ft_sensor);
			return rr_m;
		}

		sensors::GpsSensorPtr gz_gps_sensor=std::dynamic_pointer_cast<sensors::GpsSensor>(s);
		if (gz_gps_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<GpsSensorImpl>(gz_gps_sensor);
			return rr_m;
		}

		sensors::ImuSensorPtr gz_imu_sensor=std::dynamic_pointer_cast<sensors::ImuSensor>(s);
		if (gz_imu_sensor)
		{
			auto rr_m=RR_MAKE_SHARED<ImuSensorImpl>(gz_imu_sensor);
			return rr_m;
		}

		sensors::LogicalCameraSensorPtr gz_logical_camera=std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(s);
		if (gz_logical_camera)
		{
			auto rr_camera=RR_MAKE_SHARED<LogicalCameraImpl>(gz_logical_camera);
			return rr_camera;
		}

		auto rr_s=RR_MAKE_SHARED<SensorImpl>(s);
		return rr_s;
	}

	com::robotraconteur::device::DeviceInfoPtr ServerImpl::get_device_info()
	{
		device::DeviceInfoPtr ret(new device::DeviceInfo());
		ret->device = RRU::CreateIdentifierFromName("gazebosim");
		ret->manufacturer = RRU::CreateIdentifier("OSRF", "e72378a1-7f24-40f3-be19-30d5f12e2c46");
		ret->model = RRU::CreateIdentifier("Gazebo_Sim", "e90fee61-1c12-4747-b1c3-3b059b27466d");
		ret->device_classes = RR::AllocateEmptyRRList<device::DeviceClass>();
		device::DeviceClassPtr dev_class(new device::DeviceClass());
		dev_class->class_identifier = RRU::CreateIdentifier("robot_simulator","183c1fdc-6d2c-4e23-b323-6c0283b75e4c");
		dev_class->subclasses = RR::AllocateEmptyRRList<RR::RRArray<char>>();
		dev_class->subclasses->push_back(RR::stringToRRArray("physics"));
		ret->device_classes->push_back(dev_class);
		ret->user_description = "Gazebo Robot Simulator";
		return ret;
	}


}
