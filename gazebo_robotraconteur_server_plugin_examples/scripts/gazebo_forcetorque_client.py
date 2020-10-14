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

#Example client to display force torque data
#For use with rip_sensor_world.world Gazebo world
#Use rip_joint_controller.py to move the joints and see changes

import sys
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
print(server.sensor_names)
a=server.get_sensors('default::rip::joint_2::force_torque')

m=a.force_torque.PeekInValue()
print(m)

p=a.force_torque.Connect()

try:
    while True:
        if (p.InValueValid):
            print(p.InValue)
        time.sleep(.01)
except KeyboardInterrupt: pass

p.Close()
