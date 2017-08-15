#!/usr/bin/env python
#
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
import time
import os
import roslaunch
import subprocess

from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

def widget_creator(obj_ui):
    
    widget = QWidget()
    
    layout = QHBoxLayout(widget)
    layout.setSpacing(6)
    layout.setContentsMargins(0, 0, 0, 0)
    spacer_left = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
    spacer_right = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
    
    layout.addItem(spacer_left)
    layout.addWidget(obj_ui)
    layout.addItem(spacer_right)
    
    return widget

class LaunchItem:
    
    def __init__(self, launch, machine):
        
        self.launch_name = QLabel(launch)
        self.launch_name.setContentsMargins(0,0,10,0)
        self.launch_name.setMinimumHeight(40)
        
        
        self.combo_machines = QComboBox()
        self.combo_machines.setMinimumHeight(40)
        self.combo_machines.addItem('cobotgui-dev:127.0.0.1')
        self.combo_machines.addItem('cobot:192.168.0.1')
        
        rsc = os.path.join(get_pkg_dir('plugin_node_manager'),'resources')
        icon_launch = QIcon(rsc+'/launch.png')
        
        self.button_launch = QPushButton()
        self.button_launch.setIcon(icon_launch)
        self.button_launch.setIconSize(QSize(30,30))
        self.button_launch.setFixedSize(QSize(100,40))
        self.button_launch.clicked.connect(self._launch_node_slot)
        
        self.button_launch_widget = widget_creator(self.button_launch)
        
    def _launch_node_slot(self):
        
        rospy.loginfo('%s::_launch_node()'%self.launch_name.text())
        
        subprocess.Popen(['roslaunch',
                          'node_launchers',
                          self.launch_name.text()])
        
#End of file
