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

#Example client to display depth camera output
#For use with rip_sensor_world.world Gazebo world
#Use rip_joint_controller.py to move the camera

import sys
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

current_frame=None
current_depth=None

def ImageToMat(image):
    frame2=image.imageData.reshape([image.height, image.width, 3], order='C')
    return np.concatenate((np.atleast_3d(frame2[:,:,2]), np.atleast_3d(frame2[:,:,1]), np.atleast_3d(frame2[:,:,0])),axis=2)

def new_frame(pipe_ep):
    global current_frame
    global current_depth

    while (pipe_ep.Available > 0):
        image=pipe_ep.ReceivePacket()
        current_frame=ImageToMat(image)
        current_depth=image.depthData.reshape([image.height,image.width],order='C')

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
print server.SensorNames
cam=server.get_Sensors('default::rip::pendulum::depth')
image=cam.CaptureImage()
image1=ImageToMat(image)

cv2.imshow('Captured Image',image1)

depth1=image.depthData.reshape([image.height,image.width],order='C')
cv2.imshow("Captured Depth", depth1)

p=cam.ImageStream.Connect(-1)
p.PacketReceivedEvent+=new_frame

cv2.namedWindow("Image")
cv2.namedWindow("Depth")

while True:
    if (not current_frame is None):
        cv2.imshow("Image",current_frame)
    if (not current_depth is None):
        cv2.imshow("Depth", current_depth)
    ret = cv2.waitKey(50)
    if ret!=-1 and ret!=255:
        break
cv2.destroyAllWindows()

p.Close()