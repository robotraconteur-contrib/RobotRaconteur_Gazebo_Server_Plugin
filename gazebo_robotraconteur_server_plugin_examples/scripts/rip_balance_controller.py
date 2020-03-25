#!/usr/bin/env python
#
# Copyright (C) 2016 Wason Technology, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#Example controller to balance underactuated rotary inverted pendulum
#For use with rip_world.world Gazebo world

import sys
from RobotRaconteur.Client import *
import time
import math

def normalize(q):
    q1=math.fmod(q+math.pi,2*math.pi)
    if (q1 >= 0):
        return q1-math.pi
    else:
        return q1+math.pi

def swingup(q_1, q_2, u_1, u_2):
    s2=math.sin(q_2)
    c2=math.cos(q_2)

    target_energy=.48169
    total_energy=-0.0049152*c2*u_1*u_2 + 0.2408448*c2 + 0.002003024*s2**2*u_1**2 + 0.004065315*u_1**2 + 0.002005864*u_2**2 + 0.2408448

    DE=target_energy*1.5-total_energy

    f_1=.7*DE
    if (math.fabs(q_2) > math.pi*110.0/180.0):
        if (u_2 > 0):
            f_1=-f_1
    elif math.fabs(q_2) < math.pi*70.0/180.0:
        if (u_2 < 0):
            f_1=-f_1
    else:
        f_1=0
    return f_1

def main():
    global c
    c=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
    w=c.get_worlds('default')
    rip=w.get_models('rip')
    joint_1=rip.get_joints('joint_1')
    joint_2=rip.get_joints('joint_2')
    
    q_1_wire=joint_1.axes_position.Connect()
    q_2_wire=joint_2.axes_position.Connect()
    u_1_wire=joint_1.axes_velocity.Connect()
    u_2_wire=joint_2.axes_velocity.Connect()

    f_1_wire=joint_1.apply_axes_force.Connect()


    try:

        while (not q_1_wire.InValueValid) \
            or (not q_2_wire.InValueValid) \
            or (not u_1_wire.InValueValid) \
            or (not u_2_wire.InValueValid):

            time.sleep(.01)

        while True:
            q_1=normalize(q_1_wire.InValue[0])
            q_2=normalize(q_2_wire.InValue[0]+math.pi)
            u_1=u_1_wire.InValue[0]
            u_2=u_2_wire.InValue[0]
            #print str(q_1) + ", " + str(q_2) + ", " + str(u_1) + ", " + str(u_2)

            if (math.fabs(q_2) < 20.0/180.0*math.pi):
                f_1=-.1*(-2.5*q_1 + 40*q_2 + -2.5*u_1 +5*u_2)
            else:
                f_1=swingup(q_1, q_2, u_1, u_2)
            f_1_wire.OutValue=[f_1]
            time.sleep(.01)


    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
