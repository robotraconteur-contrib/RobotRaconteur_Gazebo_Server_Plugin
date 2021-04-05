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
		rrvar_depth_image_stream->SetMaxBacklog(3);
		rr_downsampler->AddPipeBroadcaster(rrvar_depth_image_stream);

		RR_SHARED_PTR<DepthCameraSensorImpl> this_ = RR_DYNAMIC_POINTER_CAST<DepthCameraSensorImpl>(shared_from_this());
		boost::weak_ptr<DepthCameraSensorImpl> weak_this = this_;
		camera_update_connection = get_camera()->DepthCamera()->ConnectNewDepthFrame([weak_this](const float *image, unsigned int width, unsigned int height, unsigned int depth, const std::string &format)
		{
			OnNewFrame(weak_this,image,width,height,format);
		});
	}

	image::ImagePtr DepthCameraSensorImpl::capture_depth_image()
	{
		return current_frame;
	}

	sensors::DepthCameraSensorPtr DepthCameraSensorImpl::get_camera()
	{
		return std::dynamic_pointer_cast<sensors::DepthCameraSensor>(get_sensor());
	}
		
	void DepthCameraSensorImpl::OnNewFrame(RR_WEAK_PTR<DepthCameraSensorImpl> this_, const float *image, unsigned int width, unsigned int height, const std::string &format)
	{
		RR_SHARED_PTR<DepthCameraSensorImpl> this1 = this_.lock();
		if (!this1) return;
		this1->OnNewFrame1(image, width, height, format);
	}

    void DepthCameraSensorImpl::OnNewFrame1(const float *image_bytes, unsigned int width, unsigned int height, const std::string &format)
	{						
		image::ImagePtr o(new image::Image());
		o->image_info = new image::ImageInfo();
		o->image_info->data_header = new com::robotraconteur::sensordata::SensorDataHeader();
		size_t image_byte_size=width*height*sizeof(float);
		o->data=RR::AttachRRArrayCopy((const uint8_t*)image_bytes,image_byte_size);
		o->image_info->encoding = image::ImageEncoding::depth_f32;
		o->image_info->width=width;
		o->image_info->height=height;
		o->image_info->step = width * sizeof(float);
		current_frame = o;
		rrvar_depth_image_stream->AsyncSendPacket(o, []() {});
	}
	  
}
