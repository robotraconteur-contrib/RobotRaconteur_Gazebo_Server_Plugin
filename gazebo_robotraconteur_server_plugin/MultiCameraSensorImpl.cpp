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

#include "MultiCameraSensorImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"
#include <gazebo/rendering/rendering.hh>

namespace RobotRaconteurGazeboServerPlugin
{
	MultiCameraSensorImpl::MultiCameraSensorImpl(sensors::MultiCameraSensorPtr gz_camera) : SensorImpl(gz_camera)
	{
		this->gz_camera = gz_camera;
	}

	void MultiCameraSensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		SensorImpl::RRServiceObjectInit(context, service_path);
		rr_downsampler->AddPipeBroadcaster(rrvar_image_stream);
		rrvar_image_stream->SetMaxBacklog(3);

		current_frame = RR::AllocateEmptyRRMap<int32_t,image::Image>();

		auto cam = get_camera();

		RR_SHARED_PTR<MultiCameraSensorImpl> this_ = RR_DYNAMIC_POINTER_CAST<MultiCameraSensorImpl>(shared_from_this());
		boost::weak_ptr<MultiCameraSensorImpl> weak_this = this_;
		for (unsigned int i=0; i<cam->CameraCount(); i++)
		{
			auto camera_update_connection1 = cam->Camera(i)->ConnectNewImageFrame([weak_this,i](const unsigned char *image, unsigned int width, unsigned int height, unsigned int depth, const std::string &format)
			{
				OnNewFrame(weak_this,image,width,height,depth,format,i);
			});
			camera_update_connection.push_back(camera_update_connection1);
		}		
	}

	int32_t MultiCameraSensorImpl::get_camera_count()
	{
		sensors::MultiCameraSensorPtr c=get_camera();
		return (int32_t)c->CameraCount();
	}
	
	image::ImagePtr MultiCameraSensorImpl::capture_image(int32_t ind)
	{

		if (ind<0) throw std::invalid_argument("Invalid camera");
		
		return current_frame->at(ind);
	}

	sensors::MultiCameraSensorPtr MultiCameraSensorImpl::get_camera()
	{
		return std::dynamic_pointer_cast<sensors::MultiCameraSensor>(get_sensor());
	}

	
	void MultiCameraSensorImpl::OnNewFrame(RR_WEAK_PTR<MultiCameraSensorImpl> this_, const unsigned char *image, unsigned int width, unsigned int height, unsigned int depth, const std::string &format, unsigned int index)
	{
		RR_SHARED_PTR<MultiCameraSensorImpl> this1 = this_.lock();
		if (!this1) return;
		this1->OnNewFrame1(image, width, height, depth, format, index);
	}

    void MultiCameraSensorImpl::OnNewFrame1(const unsigned char *image_bytes, unsigned int width, unsigned int height, unsigned int depth, const std::string &format, unsigned int index)
	{
						
		image::ImagePtr o(new image::Image());
		o->image_info = new image::ImageInfo();
		o->image_info->data_header = new com::robotraconteur::sensordata::SensorDataHeader();
		size_t image_byte_size=width*height*depth;
		o->data=RR::AttachRRArrayCopy(image_bytes,image_byte_size);
		o->image_info->encoding = detail::gz_image_enconding_to_rr_encoding(format);
		o->image_info->width=width;
		o->image_info->height=height;
		o->image_info->step = width * depth;
		(*current_frame)[index] = o;
		if (index == current_frame->size()-1)
		{
			auto o2 = RR::AllocateEmptyRRMap<int32_t, image::Image>();
			o2->GetStorageContainer() = current_frame->GetStorageContainer();
			rrvar_image_stream->AsyncSendPacket(o2, []() {});
		}
	}

}
