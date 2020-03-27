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
#include <gazebo/rendering/rendering.hh>

#include "CameraImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
	DepthCameraSensorImpl::DepthCameraSensorImpl(sensors::DepthCameraSensorPtr gz_camera) : SensorImpl(gz_camera)
	{
		this->gz_camera = gz_camera;
	}

	void DepthCameraSensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		SensorImpl::RRServiceObjectInit(context, service_path);
		rrvar_image_stream->SetMaxBacklog(3);
	}

	image::DepthImagePtr DepthCameraSensorImpl::capture_image()
	{

		sensors::DepthCameraSensorPtr c=get_camera();
		rendering::DepthCameraPtr c2=c->DepthCamera();
		if (!c2->CaptureData()) throw std::runtime_error("Image not ready");
		image::DepthImagePtr o(new image::DepthImage());
		o->depth_ticks_per_meter = 1;

		image::ImagePtr o1(new image::Image());

		const uint8_t* image_bytes=c2->ImageData();
		size_t image_byte_size=c2->ImageByteSize();
		o1->data=RR::AttachRRArrayCopy(image_bytes,image_byte_size);
		o1->image_info = new image::ImageInfo();
		o1->image_info->encoding=detail::gz_image_enconding_to_rr_encoding(c2->ImageFormat());
		o1->image_info->width=c2->ImageWidth();
		o1->image_info->height=c2->ImageHeight();
		o1->image_info->step = c2->ImageWidth() * c2->ImageDepth();
		o->intensity_image = o1;

		const float* depth_data=c2->DepthData();
		if (depth_data)
		{
			image::ImagePtr o2(new image::Image());
			o2->image_info = new image::ImageInfo();
			o2->image_info->encoding = detail::gz_image_enconding_to_rr_encoding(c2->ImageFormat());
			o2->image_info->width = c2->ImageWidth();
			o2->image_info->height = c2->ImageHeight();
			o2->image_info->step = c2->ImageWidth() * c2->ImageDepth();
			o2->image_info->encoding = image::ImageEncoding::depth_f32;
			o2->data=RR::AttachRRArrayCopy(reinterpret_cast<const uint8_t*>(depth_data),o2->image_info->width * o2->image_info->height * sizeof(float));
			o->depth_image = o2;
		}

		return o;
	}

	sensors::DepthCameraSensorPtr DepthCameraSensorImpl::get_camera()
	{
		return std::dynamic_pointer_cast<sensors::DepthCameraSensor>(get_sensor());
	}
		
	void DepthCameraSensorImpl::OnUpdate1()
	{
		SensorImpl::OnUpdate1();
		auto c = gz_camera.lock();
		if (!c) return;

		auto i=capture_image();
		rrvar_image_stream->AsyncSendPacket(i, []() {});
	}
}
