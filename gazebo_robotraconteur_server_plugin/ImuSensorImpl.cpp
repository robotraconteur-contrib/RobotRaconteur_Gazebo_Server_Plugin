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

namespace RobotRaconteurGazeboServerPlugin
{
	ImuSensorImpl::ImuSensorImpl(sensors::ImuSensorPtr gz_imu) : SensorImpl(gz_imu)
	{
		this->gz_imu = gz_imu;
	}

	static imu::ImuStatePtr gz_to_rr_imustate(sensors::ImuSensorPtr& c)
	{		
		imu::ImuStatePtr o(new imu::ImuState());
		
		auto a_v1 = c->AngularVelocity();
		auto l_a1 = c->LinearAcceleration();
		auto o_p1 = c->Orientation();

		o->angular_velocity.s.x = a_v1.X();
		o->angular_velocity.s.y = a_v1.Y();
		o->angular_velocity.s.z = a_v1.Z();

		o->linear_acceleration.s.x = l_a1.X();
		o->linear_acceleration.s.y = l_a1.Y();
		o->linear_acceleration.s.z = l_a1.Z();

		o->orientation.s.w = o_p1.W();
		o->orientation.s.x = o_p1.X();
		o->orientation.s.y = o_p1.Y();
		o->orientation.s.z = o_p1.Z();
		
		return o;
	}

	void ImuSensorImpl::RRServiceObjectInit(RR_WEAK_PTR<RR::ServerContext> context, const std::string& service_path)
	{
		SensorImpl::RRServiceObjectInit(context, service_path);

		boost::weak_ptr<ImuSensorImpl> weak_this = RR::rr_cast<ImuSensorImpl>(shared_from_this());
		this->rrvar_state->GetWire()->SetPeekInValueCallback(
			[weak_this](uint32_t ep) {
				auto this_ = weak_this.lock();
				if (!this_) throw RR::InvalidOperationException("Entity has been released");
				auto s = this_->get_imusensor();
				return gz_to_rr_imustate(s);
			}
		);
	}

	void ImuSensorImpl::setf_reference_pose()
	{
		get_imusensor()->SetReferencePose();
	}

	sensors::ImuSensorPtr ImuSensorImpl::get_imusensor()
	{
		return std::dynamic_pointer_cast<sensors::ImuSensor>(get_sensor());
	}

	void ImuSensorImpl::OnUpdate1()
	{
		SensorImpl::OnUpdate1();
		auto s = gz_imu.lock();
		if (!s) return;
		auto i = gz_to_rr_imustate(s);
		rrvar_state->SetOutValue(i);		
	}

}
