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

#Example client to change the light source color
#For use with rip_sensor_world.world Gazebo world

#Warning: Color changes do not currently show in gzclient. They
#will be applied to the camera sensors.

import sys
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

RRN.UseNumPy=True

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
w=server.get_Worlds('default')
print w.LightNames
sun=w.get_Lights('sun')

color=sun.DiffuseColor
print str(color.a) + " " + str(color.r) + " " + str(color.g) + " " + str(color.b)

color2=RRN.NewStructure('experimental.gazebo.Color',server)

color2.a=1.0
color2.r=0.0
color2.g=1.0
color2.b=0.0

sun.DiffuseColor=color2
