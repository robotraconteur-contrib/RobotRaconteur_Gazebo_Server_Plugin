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

import sys
from RobotRaconteur.Client import *
import time
import math
import numpy as np

sphere_model_sdf = """
<sdf version ="1.6">
    <model name ="sphere">
        <pose>1 0 0 0 0 0</pose>
        <link name ="link">
            <pose>0 0 .5 0 0 0</pose>
            <collision name ="collision">
            <geometry>
                <sphere><radius>0.5</radius></sphere>
            </geometry>
            </collision>
            <visual name ="visual">
            <geometry>
                <sphere><radius>0.5</radius></sphere>
            </geometry>
            </visual>
        </link>
    </model>
</sdf>
"""


server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
w=server.get_worlds('default')

pose_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", server)

model_pose = np.zeros((1,), dtype = pose_dtype)

model_pose["orientation"]["w"] = 1.0

model_pose["position"]["x"] = np.random.uniform(-2,2,1),
model_pose["position"]["y"] = np.random.uniform(-2,2,1),
model_pose["position"]["z"] = np.random.uniform(0,3,1),

w.insert_model(sphere_model_sdf, "my_sphere", model_pose)
w.get_models("my_sphere")

time.sleep(5)

w.remove_model("my_sphere")

time.sleep(1)

w.insert_model(sphere_model_sdf, "my_sphere", model_pose)



