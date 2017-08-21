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
from xml.etree import ElementTree

from pyqt_agi_extend import QtAgiCore

# Load my resources file
from cobot_gui.res import R

def set_boot_configuration(config):
    
    autorun = ElementTree.Element('boot')
    autorun.set('config', config)
    tree = ElementTree.ElementTree(autorun)
    
    tree.write(R.DIR+'/autorun.xml', encoding='utf8', method='xml')

if __name__ == "__main__":
    
    rospy.init_node('cobot_gui_autorun_%d' % os.getpid())
    
    config = rospy.get_param("~config", "${cobot_gui}/config/default.conf")
    
    set_boot_configuration(config)
    
    subprocess.Popen(['rosrun', 'cobot_gui','autorun.py'])
    
#@endcond

