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

import rospy
from roslib.packages import get_pkg_dir
import sys
import os
import subprocess
import signal
from xml.etree import ElementTree

from pyqt_agi_extend import QtAgiCore

# Load my resources file
from cobot_gui.res import R
from cobot_gui import autorun


if __name__ == "__main__":
    
    autorun.run()

    
#@endcond

