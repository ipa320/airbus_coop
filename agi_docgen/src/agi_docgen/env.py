#!/usr/bin/env python
#
# Copyright 2015 Airbus
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import rospy
from roslib.packages import get_pkg_dir
import catkin_pkg.workspaces

OUTPUT = rospy.get_param('/agi_docgen/output_path','/tmp/docu')
ROSDOC_ROOT      = get_pkg_dir("agi_docgen")# os.path.join(ROS_WS, "rosdoc")
ROSDOC_RSC       = os.path.join(OUTPUT)
ROSDOC_DOT       = os.path.join(ROSDOC_RSC, "dot")
ROSDOC_IMAGES    = os.path.join(ROSDOC_RSC, "images")
ROSDOC_POLICES   = os.path.join(ROSDOC_RSC, "polices")
ROSDOC_STYLES    = os.path.join(ROSDOC_RSC, "styles")
ROSDOC_TEMPLATES = os.path.join(ROSDOC_RSC, "templates")
ROSDOC_GEN = os.path.join(OUTPUT, "gen")
