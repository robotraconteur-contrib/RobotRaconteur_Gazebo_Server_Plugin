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

#Example client to change the light source color
#For use with rip_sensor_world.world Gazebo world

#Warning: Color changes do not currently show in gzclient. They
#will be applied to the camera sensors.

import sys
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
w=server.get_worlds('default')
print(w.light_names)
sun=w.get_lights('sun')

color=sun.diffuse_color
print(str(color[0]["a"]) + " " + str(color[0]["r"]) + " " + str(color[0]["g"]) + " " + str(color[0]["b"]))

color_dtype=RRN.GetNamedArrayDType('com.robotraconteur.color.ColorRGBAf',server)

color2=np.zeros((1,),dtype=color_dtype)

color2["a"]=1.0
color2["r"]=0.0
color2["g"]=1.0
color2["b"]=0.0

sun.diffuse_color=color2
