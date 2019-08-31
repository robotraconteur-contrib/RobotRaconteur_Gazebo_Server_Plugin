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

#include "MultiCameraSensorImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	MultiCameraSensorImpl::MultiCameraSensorImpl(sensors::MultiCameraSensorPtr gz_camera) : SensorImpl(gz_camera)
	{

	}

	void MultiCameraSensorImpl::Init()
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_camera()->ConnectUpdated(boost::bind(&MultiCameraSensorImpl::OnUpdate,c));
	}

	int32_t MultiCameraSensorImpl::get_CameraCount()
	{
		sensors::MultiCameraSensorPtr c=get_camera();
		return (int32_t)c->CameraCount();
	}
	
	image::ImagePtr MultiCameraSensorImpl::CaptureImage(int32_t ind)
	{

		if (ind<0) throw std::invalid_argument("Invalid camera");
		sensors::MultiCameraSensorPtr c=get_camera();
		rendering::CameraPtr c2=c->Camera((uint32_t)ind);
		if (!c2) throw std::invalid_argument("Invalid camera");
		if (!c2->CaptureData()) throw std::runtime_error("Image not ready");
		image::ImagePtr o(new image::Image());
		o->image_info = new image::ImageInfo();
		o->image_info->data_header = new com::robotraconteur::sensordata::SensorDataHeader();
		const uint8_t* image_bytes = c2->ImageData();
		size_t image_byte_size = c2->ImageByteSize();
		o->data = RR::AttachRRArrayCopy(image_bytes, image_byte_size);
		o->image_info->encoding = detail::gz_image_enconding_to_rr_encoding(c2->ImageFormat());
		o->image_info->width = c2->ImageWidth();
		o->image_info->height = c2->ImageHeight();
		o->image_info->step = c2->ImageWidth() * c2->ImageDepth();
		return o;
	}

	sensors::MultiCameraSensorPtr MultiCameraSensorImpl::get_camera()
	{
		return std::dynamic_pointer_cast<sensors::MultiCameraSensor>(get_sensor());
	}

	void MultiCameraSensorImpl::set_ImageStream(RR::PipePtr<RR::RRMapPtr<int32_t,image::Image> > value)
	{
		boost::mutex::scoped_lock lock(MultiCameraSensor_default_impl::this_lock);
		rrvar_ImageStream = RR_MAKE_SHARED<RR::PipeBroadcaster<RR::RRMapPtr<int32_t, image::Image> > >();
		rrvar_ImageStream->Init(value, 3);
	}

	void MultiCameraSensorImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<MultiCameraSensorImpl> c1=RR_DYNAMIC_POINTER_CAST<MultiCameraSensorImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	void MultiCameraSensorImpl::OnUpdate1()
	{
		RR::PipeBroadcasterPtr<RR::RRMapPtr<int32_t, image::Image> > b;
		{
		boost::mutex::scoped_lock lock(MultiCameraSensor_default_impl::this_lock);
		b=rrvar_ImageStream;
		}
		if (b)
		{
			auto o=RR::AllocateEmptyRRMap<int32_t, image::Image >();

			int32_t count=get_CameraCount();
			for (int32_t i=0; i<count; i++ )
			{
				auto img=CaptureImage(i);
				o->insert(std::make_pair(i,img));
			}
			b->AsyncSendPacket(o, []() {});
		}

	}
}