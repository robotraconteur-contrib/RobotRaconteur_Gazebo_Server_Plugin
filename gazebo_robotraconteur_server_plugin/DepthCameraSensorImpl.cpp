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

#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

#include "CameraImpl.h"

namespace RobotRaconteurGazeboServerPlugin
{
	DepthCameraSensorImpl::DepthCameraSensorImpl(sensors::DepthCameraSensorPtr gz_camera) : SensorImpl(gz_camera)
	{

	}

	void DepthCameraSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_camera()->ConnectUpdated(boost::bind(&DepthCameraSensorImpl::OnUpdate,c));
	}

	image::DepthImagePtr DepthCameraSensorImpl::CaptureImage()
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
		
	void DepthCameraSensorImpl::set_ImageStream(RR::PipePtr<image::DepthImagePtr> value)
	{
		boost::mutex::scoped_lock lock(DepthCameraSensor_default_impl::this_lock);				
		rrvar_ImageStream=RR_MAKE_SHARED<RR::PipeBroadcaster<image::DepthImagePtr> >();
		rrvar_ImageStream->Init(value,3);
	}

	void DepthCameraSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<DepthCameraSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<DepthCameraSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	void DepthCameraSensorImpl::OnUpdate1()
	{

		RR::PipeBroadcasterPtr<image::DepthImagePtr> b;
		{
		boost::mutex::scoped_lock lock(DepthCameraSensor_default_impl::this_lock);
		b=rrvar_ImageStream;
		}
		if (b)
		{
			auto i=CaptureImage();
			b->AsyncSendPacket(i, []() {});
		}

	}
}
