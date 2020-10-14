#!/usr/bin/env python
#
# Copyright (C) 2016-2020 Wason Technology, LLC
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

#Example using Gazebo joint PID controller
#For use with rip_world.world Gazebo world

import sys
from RobotRaconteur.Client import *
import time
import math

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
print(server.world_names)
w=server.get_worlds('default')
print(w.name)
m=w.get_models('rip')
try:
    m.destroy_joint_controller()
except: pass
m.create_joint_controller()
c=m.get_joint_controller()
c.add_joint('rip::joint_1')
c.add_joint('rip::joint_2')

pid=RRN.NewStructure('com.robotraconteur.pid.PIDParam', c)
pid.p=10
pid.i=0
pid.d=.2
pid.imax=100
pid.imin=-100
pid.cmdMax=1000
pid.cmdMin=-1000

print(c.joint_names)
print(c.position_pid)
c.setf_position_pid('rip::joint_1',pid)
c.setf_position_pid('rip::joint_2',pid)


try:
    t_start=time.time()
    while True:

        t1=time.time()-t_start
        t=math.fmod(t1,20.0)

        theta1=0.0
        theta2=0.0
        if (t <= 5.0):
            theta1=-0.5*math.pi
            theta2=0.5*math.pi + t/5.0*math.pi
        if (t > 5.0 and t <= 10.0):
            theta1=-0.5*math.pi + (t-5.0)/5.0*math.pi
            theta2=1.5*math.pi
        if (t > 10.0 and t <= 15.0):
            theta1=0.5*math.pi
            theta2=1.5*math.pi-(t-10.0)/5.00*math.pi
        if (t > 15.0):
            theta1=0.5*math.pi - (t-15.0)/5.0*math.pi
            theta2=0.5*math.pi
        c.joint_position_command.PokeOutValue({'rip::joint_1': theta1, 'rip::joint_2': theta2})
        time.sleep(.01)

except KeyboardInterrupt: pass

