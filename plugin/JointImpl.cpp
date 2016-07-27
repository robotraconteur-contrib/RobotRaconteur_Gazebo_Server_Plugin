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

#include "JointImpl.h"
#include "robotraconteur_gazebo_server_plugin.h"

namespace RobotRaconteurGazeboServerPlugin
{
	JointImpl::JointImpl(physics::JointPtr j)
	{
		gz_joint=j;
		link_name=j->GetParent()->GetName();
		model_name=j->GetParent()->GetParentModel()->GetName();
		gz_world=j->GetParent()->GetParentModel()->GetWorld();

		axes_forces=RR::AllocateRRArray<double>(j->GetAngleCount());
	}

	void JointImpl::Init()
	{
		RR_WEAK_PTR<JointImpl> j1=shared_from_this();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		          boost::bind(&JointImpl::OnUpdate, j1, _1));
	}

	void JointImpl::OnUpdate(RR_WEAK_PTR<JointImpl> j, const common::UpdateInfo & _info)
    {
		RR_SHARED_PTR<JointImpl> j1=j.lock();
		if (!j1) return;
		j1->OnUpdate1(_info);
    }

	void JointImpl::OnUpdate1(const common::UpdateInfo & _info)
	{

		//std::cout << _info.simTime.Double() << std::endl;


		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > axisangle_b;
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > axisvel_b;
		RR_SHARED_PTR<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > ft_b;

		unsigned int axis_count;
		physics::JointPtr j=get_joint();
		{
			boost::mutex::scoped_lock lock(this_lock);

			axis_count=j->GetAngleCount();

			try
			{
				if (m_AxisPositionSetWire_conn)
				{
					if (m_AxisPositionSetWire_conn->GetInValueValid())
					{
						RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double> > > in_val=m_AxisPositionSetWire_conn->GetInValue();
						for(std::map<int32_t,RR_SHARED_PTR<RR::RRArray<double> > >::iterator e=in_val->map.begin(); e!=in_val->map.end(); e++)
						{
							if (e->first < axis_count)
							{
								j->SetPosition(e->first, RR::RRArrayToScalar(e->second));
							}
						}
					}
				}
			}
			catch (std::exception&) {}

			try
			{
				if (m_AxisVelocitySetWire_conn)
				{
					if (m_AxisVelocitySetWire_conn->GetInValueValid())
					{
						RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double> > > in_val=m_AxisVelocitySetWire_conn->GetInValue();
						for(std::map<int32_t,RR_SHARED_PTR<RR::RRArray<double> > >::iterator e=in_val->map.begin(); e!=in_val->map.end(); e++)
						{
							if (e->first < axis_count)
							{
								j->SetVelocity(e->first, RR::RRArrayToScalar(e->second));
							}
						}
					}
				}
			}
			catch (std::exception&) {}

			bool force_set=false;

			try
			{
				if (m_ForceSetWire_conn)
				{
					if (m_ForceSetWire_conn->GetInValueValid())
					{
						force_set=true;
						RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double> > > in_val=m_ForceSetWire_conn->GetInValue();
						for(std::map<int32_t,RR_SHARED_PTR<RR::RRArray<double> > >::iterator e=in_val->map.begin(); e!=in_val->map.end(); e++)
						{
							if (e->first < axis_count)
							{
								j->SetForce(e->first, RR::RRArrayToScalar(e->second));
							}
						}
					}
				}
			}
			catch (std::exception&) {}



			if (!force_set)
			{
				for (unsigned int i=0; i < axes_forces->size(); i++ )
				{
					if (i<axis_count)
					{
						j->SetForce(i, (*axes_forces)[i]);
					}
				}
			}

			axisangle_b=m_AxisAngleGetWire_b;
			axisvel_b=m_AxisVelocityGetWire_b;
			ft_b=m_ForceTorqueGetWire_b;

		}
		RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > axisangle=RR_MAKE_SHARED<RR::RRMap<int32_t,RR::RRArray<double > > >();
		RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > axisvel=RR_MAKE_SHARED<RR::RRMap<int32_t,RR::RRArray<double > > >();
		RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > ft=RR_MAKE_SHARED<RR::RRMap<int32_t,RR::RRArray<double > > >();

		for (unsigned int i=0; i<axis_count; i++)
		{
			axisangle->map.insert(std::make_pair((int32_t)i,RR::ScalarToRRArray(GetAxisAngle(i))));
			axisvel->map.insert(std::make_pair((int32_t)i,RR::ScalarToRRArray(GetAxisVelocity(i))));
		}

		//TODO: Determine "Provide Feedback?"
		//ft->map.insert(std::make_pair(0,GetForceTorque(0)));
		//ft->map.insert(std::make_pair(1,GetForceTorque(1)));



		if (axisangle_b) axisangle_b->SetOutValue(axisangle);
		if (axisvel_b) axisvel_b->SetOutValue(axisvel);
		//if (ft_b) ft_b->SetOutValue(ft);

	}

	physics::JointPtr JointImpl::get_joint()
	{
		physics::JointPtr j=gz_joint.lock();
		if (!j) throw std::runtime_error("Joint has been released");
		return j;
	}

	std::string JointImpl::get_Name()
	{
		return get_joint()->GetName();
	}
	void JointImpl::set_Name(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	std::string JointImpl::get_ScopedName()
	{
		return get_joint()->GetScopedName(true);
	}
	void JointImpl::set_ScopedName(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	std::string JointImpl::get_ParentLinkName()
	{
		return get_joint()->GetParent()->GetName();
	}
	void JointImpl::set_ParentLinkName(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	std::string JointImpl::get_ChildLinkName()
	{
		return get_joint()->GetChild()->GetName();
	}
	void JointImpl::set_ChildLinkName(std::string value)
	{
		throw std::runtime_error("Read only property");
	}

	int32_t JointImpl::get_AxisCount()
	{
		return get_joint()->GetAngleCount();
	}
	void JointImpl::set_AxisCount(int32_t value)
	{
		throw std::runtime_error("Read only property");
	}

	RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > JointImpl::get_AxesAngles()
	{
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > o=RR_MAKE_SHARED<RR::RRMap<int32_t,RR::RRArray<double > > >();
		for (int32_t i=0; i<axis_count; i++)
		{
			double v=j->GetAngle(i).Radian();
			o->map.insert(std::make_pair(i,RR::ScalarToRRArray(v)));
		}
		return o;
	}
	void JointImpl::set_AxesAngles(RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > value)
	{
		RR_NULL_CHECK(value);
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		for (std::map<int32_t,RR_SHARED_PTR<RR::RRArray<double > > >::iterator e=value->map.begin(); e!=value->map.end(); e++)
		{
			RR_NULL_CHECK(e->second);
			if (e->first > axis_count)
			{
				throw std::invalid_argument("Invalid axis");
			}
			double v=0;
			v=RR::RRArrayToScalar(e->second);
			j->SetPosition((unsigned int)e->first, v);
		}
	}

	RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > JointImpl::get_AxesVelocities()
	{
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > o=RR_MAKE_SHARED<RR::RRMap<int32_t,RR::RRArray<double > > >();
		for (int32_t i=0; i<axis_count; i++)
		{
			double v=j->GetVelocity(i);
			o->map.insert(std::make_pair(i,RR::ScalarToRRArray(v)));
		}
		return o;
	}
	void JointImpl::set_AxesVelocities(RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > value)
	{
		RR_NULL_CHECK(value);
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		for (std::map<int32_t,RR_SHARED_PTR<RR::RRArray<double > > >::iterator e=value->map.begin(); e!=value->map.end(); e++)
		{
			RR_NULL_CHECK(e->second);
			if (e->first > axis_count)
			{
				throw std::invalid_argument("Invalid axis");
			}
			double v=0;
			v=RR::RRArrayToScalar(e->second);
			j->SetVelocity((unsigned int)e->first, v);
		}
	}

	RR_SHARED_PTR<RR::RRArray<double > > JointImpl::GetGlobalAxis(int32_t axis)
	{
		RR_SHARED_PTR<RR::RRArray<double> > o=RR::AllocateRRArray<double>(3);
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");

		math::Vector3 v=j->GetGlobalAxis(axis);
		(*o)[0]=v.x; (*o)[1]=v.y; (*o)[2]=v.z;
		return 0;
	}

	RR_SHARED_PTR<RR::RRArray<double > > JointImpl::GetLocalAxis(int32_t axis)
	{
		RR_SHARED_PTR<RR::RRArray<double> > o=RR::AllocateRRArray<double>(3);
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");

		math::Vector3 v=j->GetLocalAxis(axis);
		(*o)[0]=v.x; (*o)[1]=v.y; (*o)[2]=v.z;
		return 0;
	}

	double JointImpl::GetAxisAngle(int32_t axis)
	{
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		return j->GetAngle(axis).Radian();
	}

	void JointImpl::SetAxisPosition(int32_t axis, double value)
	{
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		j->SetPosition(axis,value);
	}

	double JointImpl::GetAxisVelocity(int32_t axis)
	{
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		return j->GetVelocity(axis);
	}

	void JointImpl::SetAxisVelocity(int32_t axis, double value)
	{
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		j->SetVelocity(axis,value);
	}

	double JointImpl::GetForce(int32_t axis)
	{
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");
		return j->GetForce(axis);
	}

	void JointImpl::SetForce(int32_t axis, double value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		physics::JointPtr j=get_joint();
		int axis_count=j->GetAngleCount();
		if (axis > axis_count) throw std::invalid_argument("Invalid axis");

		if (axes_forces->size() < axis_count)
		{
			RR_SHARED_PTR<RR::RRArray<double> > axes_forces1=RR::AllocateRRArray<double>(axis_count);
			for (size_t i=0; i<axes_forces->size(); i++)
			{
				(*axes_forces1)[i]=(*axes_forces)[i];
			}
			axes_forces=axes_forces1;
		}

		(*axes_forces)[axis]=value;
	}

	RR_SHARED_PTR<RR::RRArray<double > > JointImpl::GetForceTorque(int32_t link)
	{
		RR_SHARED_PTR<RR::RRArray<double> > o=RR::AllocateRRArray<double>(6);

		physics::JointWrench a=get_joint()->GetForceTorque(0);

		if (link==0)
		{
			(*o)[0]=a.body1Torque.x; (*o)[1]=a.body1Torque.y; (*o)[2]=a.body1Torque.z;
			(*o)[3]=a.body1Force.x; (*o)[4]=a.body1Force.y; (*o)[5]=a.body1Force.z;
		}
		else if (link==1)
		{
			(*o)[0]=a.body2Torque.x; (*o)[1]=a.body2Torque.y; (*o)[2]=a.body2Torque.z;
			(*o)[3]=a.body2Force.x; (*o)[4]=a.body2Force.y; (*o)[5]=a.body2Force.z;
		}
		else
		{
			throw std::invalid_argument("Invalid link");
		}
		return o;
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > JointImpl::get_AxisAngleGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_AxisAngleGetWire;
	}
	void JointImpl::set_AxisAngleGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_AxisAngleGetWire) throw std::runtime_error("Already set");
		m_AxisAngleGetWire=value;
		m_AxisAngleGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > > >();
		m_AxisAngleGetWire_b->Init(m_AxisAngleGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > JointImpl::get_AxisVelocityGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_AxisVelocityGetWire;
	}
	void JointImpl::set_AxisVelocityGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_AxisVelocityGetWire) throw std::runtime_error("Already set");
		m_AxisVelocityGetWire=value;
		m_AxisVelocityGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > > >();
		m_AxisVelocityGetWire_b->Init(m_AxisVelocityGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > JointImpl::get_ForceTorqueGetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_ForceTorqueGetWire;
	}
	void JointImpl::set_ForceTorqueGetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_ForceTorqueGetWire) throw std::runtime_error("Already set");
		m_ForceTorqueGetWire=value;
		m_ForceTorqueGetWire_b=RR_MAKE_SHARED<RR::WireBroadcaster<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double > > > > >();
		m_ForceTorqueGetWire_b->Init(m_ForceTorqueGetWire);
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > JointImpl::get_AxisPositionSetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_AxisPositionSetWire;
	}
	void JointImpl::set_AxisPositionSetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_AxisPositionSetWire) throw std::runtime_error("Already set");
		m_AxisPositionSetWire=value;
		RR_WEAK_PTR<JointImpl> l=boost::dynamic_pointer_cast<JointImpl>(shared_from_this());
		m_AxisPositionSetWire->SetWireConnectCallback(boost::bind(&JointImpl::OnAxisPositionSetWireConnect,l,_1));
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > JointImpl::get_AxisVelocitySetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_AxisVelocitySetWire;
	}
	void JointImpl::set_AxisVelocitySetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_AxisVelocitySetWire) throw std::runtime_error("Already set");
		m_AxisVelocitySetWire=value;
		RR_WEAK_PTR<JointImpl> l=boost::dynamic_pointer_cast<JointImpl>(shared_from_this());
		m_AxisVelocitySetWire->SetWireConnectCallback(boost::bind(&JointImpl::OnAxisVelocitySetWireConnect,l,_1));
	}

	RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > JointImpl::get_ForceSetWire()
	{
		boost::mutex::scoped_lock lock(this_lock);
		return m_ForceSetWire;
	}
	void JointImpl::set_ForceSetWire(RR_SHARED_PTR<RR::Wire<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > value)
	{
		boost::mutex::scoped_lock lock(this_lock);
		if (m_ForceSetWire) throw std::runtime_error("Already set");
		m_ForceSetWire=value;
		RR_WEAK_PTR<JointImpl> l=boost::dynamic_pointer_cast<JointImpl>(shared_from_this());
		m_ForceSetWire->SetWireConnectCallback(boost::bind(&JointImpl::OnForceSetWireConnect,l,_1));
	}

	void JointImpl::OnAxisPositionSetWireConnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection)
	{
		RR_SHARED_PTR<JointImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		//if(l1->m_AxisPositionSetWire_conn) throw std::runtime_error("Wire in use");
		l1->m_AxisPositionSetWire_conn=connection;
		l1->m_AxisPositionSetWire_conn->SetWireConnectionClosedCallback(boost::bind(&JointImpl::OnAxisPositionSetWireDisconnect,l,_1));

	}
	void JointImpl::OnAxisPositionSetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection)
	{
		RR_SHARED_PTR<JointImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		if(l1->m_AxisPositionSetWire_conn==connection)
		{
			l1->m_AxisPositionSetWire_conn.reset();
		}
	}
	void JointImpl::OnAxisVelocitySetWireConnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection)
	{
		RR_SHARED_PTR<JointImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		//if(l1->m_AxisVelocitySetWire_conn) throw std::runtime_error("Wire in use");
		l1->m_AxisVelocitySetWire_conn=connection;
		l1->m_AxisVelocitySetWire_conn->SetWireConnectionClosedCallback(boost::bind(&JointImpl::OnAxisVelocitySetWireDisconnect,l,_1));

	}
	void JointImpl::OnAxisVelocitySetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection)
	{
		RR_SHARED_PTR<JointImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		if(l1->m_AxisVelocitySetWire_conn==connection)
		{
			l1->m_AxisVelocitySetWire_conn.reset();
		}
	}
	void JointImpl::OnForceSetWireConnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection)
	{
		RR_SHARED_PTR<JointImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		//if(l1->m_ForceSetWire_conn) throw std::runtime_error("Wire in use");
		l1->m_ForceSetWire_conn=connection;
		l1->m_ForceSetWire_conn->SetWireConnectionClosedCallback(boost::bind(&JointImpl::OnForceSetWireDisconnect,l,_1));
	}
	void JointImpl::OnForceSetWireDisconnect(RR_WEAK_PTR<JointImpl> l, RR_SHARED_PTR<RR::WireConnection<RR_SHARED_PTR<RR::RRMap<int32_t,RR::RRArray<double >  > > > > connection)
	{
		RR_SHARED_PTR<JointImpl> l1=l.lock();
		if (!l1) return;
		boost::mutex::scoped_lock lock(l1->this_lock);
		if(l1->m_ForceSetWire_conn==connection)
		{
			l1->m_ForceSetWire_conn.reset();
		}
	}
}
