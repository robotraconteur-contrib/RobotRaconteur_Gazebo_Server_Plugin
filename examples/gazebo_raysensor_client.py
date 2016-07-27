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

#Example client to display ray sensor data
#For use with rip_sensor_world.world Gazebo world
#Use rip_joint_controller.py to move the camera

import sys
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

RRN.UseNumPy=True

current_ranges=None
current_intensities=None

def new_frame(pipe_ep):
    global current_ranges
    global current_intensities

    while (pipe_ep.Available > 0):
        scan=pipe_ep.ReceivePacket()
        current_ranges=scan.ranges.reshape([scan.verticalCount,scan.count],order='C')
        current_intensities=scan.intensities.reshape([scan.verticalCount,scan.count],order='C')


server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
print server.SensorNames
cam=server.get_Sensors('default::rip::pendulum::laser')
scan=cam.CaptureScan()

depth1=scan.ranges.reshape([scan.verticalCount,scan.count],order='C')
cv2.imshow("Captured Depth", depth1)

intensity1=scan.intensities.reshape([scan.verticalCount,scan.count],order='C')
cv2.imshow("Captured Intensity", intensity1)

p=cam.ScanStream.Connect(-1)
p.PacketReceivedEvent+=new_frame

cv2.namedWindow("Ranges")
cv2.namedWindow("Intensities")

while True:
    if (not current_ranges is None):
        cv2.imshow("Ranges",current_ranges)
    if (not current_intensities is None):
        cv2.imshow("Intensities", current_intensities)
    if cv2.waitKey(50)!=-1:
        break
cv2.destroyAllWindows()

p.Close()
