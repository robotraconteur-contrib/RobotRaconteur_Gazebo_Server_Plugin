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

#include "CameraImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	CameraImpl::CameraImpl(sensors::CameraSensorPtr gz_camera) : SensorImpl(gz_camera)
	{
		this->gz_camera = gz_camera;		
	}

	void CameraImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		SensorImpl::RRServiceObjectInit(context, service_path);

		rrvar_image_stream->SetMaxBacklog(3);
		rr_downsampler->AddPipeBroadcaster(rrvar_image_stream);
		RR_SHARED_PTR<CameraImpl> this_ = RR_DYNAMIC_POINTER_CAST<CameraImpl>(shared_from_this());
		boost::weak_ptr<CameraImpl> weak_this = this_;
		camera_update_connection = get_camera()->Camera()->ConnectNewImageFrame([weak_this](const unsigned char *image, unsigned int width, unsigned int height, unsigned int depth, const std::string &format)
		{
			OnNewFrame(weak_this,image,width,height,depth,format);
		});
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
			else if (format == "L16" || format == "L_INT16" || format == "L_UINT16")
			{
				return mono16;
			}
			else if (format == "R8G8B8" || format == "RGB_INT8")
			{
				return rgb888;
			}
			else if (format == "B8G8R8" || format == "BGR_INT8")
			{
				return bgr888;
			}
			else if (format == "BAYER_RGGB8")
			{
				return bayer_rggb8888;
			}
			else if (format == "BAYER_BGGR8")
			{
				return bayer_bggr8888;
			}
			else if (format == "BAYER_GBRG8")
			{
				return bayer_gbrg8888;
			}
			else if (format == "BAYER_GRBG8")
			{
				return bayer_grbg8888;
			}
			else if (format == "R_FLOAT16")
			{
				return mono_f16;
			}
			else if (format == "R_FLOAT32")
			{
				return mono_f32;
			}
			else
			{
				return unknown;
			}
		}
	}

	image::ImagePtr CameraImpl::capture_image()
	{
		return current_frame;
		
	}

	sensors::CameraSensorPtr CameraImpl::get_camera()
	{
		return std::dynamic_pointer_cast<sensors::CameraSensor>(get_sensor());
	}
		
	void CameraImpl::OnNewFrame(RR_WEAK_PTR<CameraImpl> this_, const unsigned char *image, unsigned int width, unsigned int height, unsigned int depth, const std::string &format)
	{
		RR_SHARED_PTR<CameraImpl> this1 = this_.lock();
		if (!this1) return;
		this1->OnNewFrame1(image, width, height, depth, format);
	}

	void CameraImpl::OnNewFrame1(const unsigned char *image_bytes, unsigned int width, unsigned int height, unsigned int depth, const std::string &format)
	{
		sensors::CameraSensorPtr c=get_camera();
				
		image::ImagePtr o(new image::Image());
		o->image_info = new image::ImageInfo();
		o->image_info->data_header = new com::robotraconteur::sensordata::SensorDataHeader();
		size_t image_byte_size=width*height*depth;
		o->data=RR::AttachRRArrayCopy(image_bytes,image_byte_size);
		o->image_info->encoding = detail::gz_image_enconding_to_rr_encoding(format);
		o->image_info->width=width;
		o->image_info->height=height;
		o->image_info->step = width * depth;
		current_frame = o;
		rrvar_image_stream->AsyncSendPacket(o, []() {});
	}
}
