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

#Example client to print out contact sensor data
#For use with rip_sensor_world.world Gazebo world
#Use rip_joint_controller.py to move the camera

import sys
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

RRN.UseNumPy=True

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
print server.SensorNames
contact=server.get_Sensors('default::rip::arm::contact_sensor')
contacts=contact.CaptureContacts()


contacts=contact.CaptureContacts()
if (len(contacts) > 0):
    print contacts[0].contactName1



p=contact.ContactWire.Connect()

try:
    while True:
        if (p.InValueValid):
            contacts=p.InValue
            if (len(contacts) > 0):
                print contacts[0].contactName1
        time.sleep(.01)
except KeyboardInterrupt: pass


p.Close()
