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

#include "CameraImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	CameraImpl::CameraImpl(sensors::CameraSensorPtr gz_camera) : SensorImpl(gz_camera)
	{

	}

	void CameraImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		RR_WEAK_PTR<SensorImpl> c=shared_from_this();
		updateConnection=get_camera()->ConnectUpdated(boost::bind(&CameraImpl::OnUpdate,c));

		rrvar_image_stream->SetMaxBacklog(3);
	}

	namespace detail
	{

		image::ImageEncoding::ImageEncoding gz_image_enconding_to_rr_encoding(const std::string& format)
		{
			using namespace image::ImageEncoding;
			if (format == "L8" || format == "L_INT8")
			{
				return mono8;
			}
			else if (format == "L16" || format == "L_INT16")
			{
				return mono16;
			}
			else if (format == "R8G8B8" || format == "RGB_INT8")
			{
				return rgb8;
			}
			else if (format == "B8G8R8" || format == "BGR_INT8")
			{
				return bgr8;
			}
			else if (format == "BAYER_RGGB8")
			{
				return bayer_rggb8;
			}
			else if (format == "BAYER_BGGR8")
			{
				return bayer_bggr8;
			}
			else if (format == "BAYER_GBRG8")
			{
				return bayer_gbrg8;
			}
			else if (format == "BAYER_GRBG8")
			{
				return bayer_grbg8;
			}
			else
			{
				return unknown;
			}
		}
	}

	image::ImagePtr CameraImpl::capture_image()
	{

		sensors::CameraSensorPtr c=get_camera();
		rendering::CameraPtr c2=c->Camera();
		if (!c2->CaptureData()) throw std::runtime_error("Image not ready");
		image::ImagePtr o(new image::Image());
		o->image_info = new image::ImageInfo();
		o->image_info->data_header = new com::robotraconteur::sensordata::SensorDataHeader();
		const uint8_t* image_bytes=c2->ImageData();
		size_t image_byte_size=c2->ImageByteSize();
		o->data=RR::AttachRRArrayCopy(image_bytes,image_byte_size);
		o->image_info->encoding = detail::gz_image_enconding_to_rr_encoding(c2->ImageFormat());
		o->image_info->width=c2->ImageWidth();
		o->image_info->height=c2->ImageHeight();
		o->image_info->step = c2->ImageWidth() * c2->ImageDepth();
		return o;
	}

	sensors::CameraSensorPtr CameraImpl::get_camera()
	{
		return std::dynamic_pointer_cast<sensors::CameraSensor>(get_sensor());
	}
		
	void CameraImpl::OnUpdate(RR_WEAK_PTR<SensorImpl> c)
	{
		RR_SHARED_PTR<CameraImpl> c1=RR_DYNAMIC_POINTER_CAST<CameraImpl>(c.lock());
		if (!c1) return;
		c1->OnUpdate1();
	}

	void CameraImpl::OnUpdate1()
	{
		auto i=capture_image();
		rrvar_image_stream->AsyncSendPacket(i, []() {});
	}
}
