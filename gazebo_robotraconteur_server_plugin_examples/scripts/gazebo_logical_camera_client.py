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

#Example client to display camera output
#For use with rip_world.world or rip_sensor_world.world Gazebo worlds
#Use rip_joint_controller.py to move the camera

import sys
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

current_logical_image=None

def print_logical_image(img):
    print("Detected " + str(len(img.recognized_objects)))
    for e in img.recognized_objects:
        print("  Detected object: " + str(e.recognized_object.name))
        e_pos = e.pose.pose.pose["position"]
        print("  Detected object position: (%d,%d,%d)" % (e_pos["x"],e_pos["y"],e_pos["z"]))

def new_logical_image(pipe_ep):
    global current_logical_image

    while (pipe_ep.Available > 0):
        current_logical_image=pipe_ep.ReceivePacket()
        

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
print(server.sensor_names)
cam=server.get_sensors('default::rip::pendulum::logical_camera')
image=cam.capture_image()
print_logical_image(image)


p=cam.image_stream.Connect(-1)
p.PacketReceivedEvent+=new_logical_image

try:
    while True:
        if current_logical_image is not None:
            print_logical_image(current_logical_image)
        time.sleep(.05)
except KeyboardInterrupt: pass

p.Close()


