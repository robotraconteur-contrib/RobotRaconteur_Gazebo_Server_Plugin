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

#include "LogicalCameraImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

#include "RobotRaconteurCompanion/Util/IdentifierUtil.h"

namespace RRU = RobotRaconteur::Companion::Util;

namespace RobotRaconteurGazeboServerPlugin
{
	LogicalCameraImpl::LogicalCameraImpl(sensors::LogicalCameraSensorPtr gz_camera) : SensorImpl(gz_camera)
	{
		this->gz_camera = gz_camera;
	}

	void LogicalCameraImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		SensorImpl::RRServiceObjectInit(context, service_path);

		rrvar_image_stream->SetMaxBacklog(3);
		rr_downsampler->AddPipeBroadcaster(rrvar_image_stream);
	}

	
	objrec::RecognizedObjectsPtr LogicalCameraImpl::capture_image()
	{

		sensors::LogicalCameraSensorPtr c=get_camera();
		auto gz_img = c->Image();		
		objrec::RecognizedObjectsPtr ret(new objrec::RecognizedObjects());
		ret->recognized_objects = RR::AllocateEmptyRRList<objrec::RecognizedObject>();
		for (auto m : gz_img.model())
		{
			objrec::RecognizedObjectPtr ret1(new objrec::RecognizedObject());
			ret1->recognized_object = RRU::CreateIdentifierFromName(m.name());
			geometry::NamedPoseWithCovariancePtr ret1_pose_w_cov(new geometry::NamedPoseWithCovariance());
			ret1_pose_w_cov->pose.reset(new geometry::NamedPose());
			ret1_pose_w_cov->pose->pose.s.position.s.x = m.pose().position().x();
			ret1_pose_w_cov->pose->pose.s.position.s.y = m.pose().position().y();
			ret1_pose_w_cov->pose->pose.s.position.s.z = m.pose().position().z();
			ret1_pose_w_cov->pose->pose.s.orientation.s.w = m.pose().orientation().w();
			ret1_pose_w_cov->pose->pose.s.orientation.s.x = m.pose().orientation().x();
			ret1_pose_w_cov->pose->pose.s.orientation.s.y = m.pose().orientation().y();
			ret1_pose_w_cov->pose->pose.s.orientation.s.z = m.pose().orientation().z();
			ret1_pose_w_cov->covariance = RR::AllocateEmptyRRMultiDimArray<double>({6,6});
			
			ret1->pose = ret1_pose_w_cov;
			ret->recognized_objects->push_back(ret1);			
		}
		return ret;		
	}

	sensors::LogicalCameraSensorPtr LogicalCameraImpl::get_camera()
	{
		return std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(get_sensor());
	}
		
	void LogicalCameraImpl::OnUpdate1()
	{
		SensorImpl::OnUpdate1();

		auto c = gz_camera.lock();
		if (!c) return;
		auto i=capture_image();
		rrvar_image_stream->AsyncSendPacket(i, []() {});
	}
}
