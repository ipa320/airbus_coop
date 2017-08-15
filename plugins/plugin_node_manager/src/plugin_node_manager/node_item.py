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
import rosnode
import roslaunch
import subprocess
from roslaunch import main as ros_launch

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

class NodeItem:
    
    NONE = 'None'
    RUNNING = 'Running'
    ABARTED = 'Aborted'
    SHUTDOWN = 'Shutdown'
    
    NODE_EXCEPTION = ['/rosout']
    
    def __init__(self, node_uri, node_name):
        
        rsc = os.path.join(get_pkg_dir('plugin_node_manager'),'resources')
        
        self._icon_node_start = QIcon(rsc+'/start.png')
        self._icon_node_stop = QIcon(rsc+'/stop.png')
        
        self.uri = QLabel(node_uri)
        self.uri.setContentsMargins(0,0,10,0)
        self.uri.setMinimumHeight(40)
        
        self.name = QLabel(node_name)
        self.name.setContentsMargins(0,0,10,0)
        self.name.setMinimumHeight(40)
        
        self.status = QLabel(self.RUNNING)
        self.status.setStyleSheet('qproperty-alignment: AlignCenter;')
        self.status.setMinimumSize(QSize(100,40))
        
        self.ping = QLabel('...')
        self.ping.setStyleSheet("qproperty-alignment: AlignCenter;")
        
        self.button_start_stop = QPushButton()
        self.button_start_stop.setIcon(self._icon_node_stop)
        self.button_start_stop.setIconSize(QSize(30,30))
        self.button_start_stop.setFixedSize(QSize(100,40))
        
        self.button_start_stop_widget = widget_creator(self.button_start_stop)
        
        if node_name not in self.NODE_EXCEPTION:
            self.button_start_stop.clicked.connect(self.start_stop_slot)
            self.button_start_stop.setEnabled(False)
        
        self.current_status = self.NONE
        
    def start_stop_slot(self):
        
        self.button_start_stop.setEnabled(False)
        
        if self.current_status == self.RUNNING:
            self.stop_node()
        else:
            self.start_node()
        
    def start_node(self):
        rospy.loginfo('%s::started()'%self.name.text())
        
        launch_file = self.name.text().replace('/','')
        launch_file += '.launch'
        subprocess.Popen(['roslaunch',
                          'node_launchers',
                          launch_file])
        
    def stop_node(self):
        rospy.loginfo('%s::stoped()'%self.name.text())
        rosnode._rosnode_cmd_kill(['fake','fake',self.name.text()])
        
    def refresh(self, status, ping=None):
        
        if ping is not None:
            self.ping.setText(str("%.3f"%ping))
        else:
            self.ping.setText('...')
        
        if status != self.current_status:
            self.current_status = status
            self.button_start_stop.setEnabled(True)
            self.status.setText(self.current_status)
            if self.current_status == self.RUNNING:
                self.status.setStyleSheet("background:rgb(0,255,0);")
                self.button_start_stop.setIcon(self._icon_node_stop)
            elif self.current_status == self.ABARTED:
                self.status.setStyleSheet("background:rgb(255,0,0);")
                self.button_start_stop.setIcon(self._icon_node_start)
            elif self.current_status == self.SHUTDOWN:
                self.status.setStyleSheet("background:rgb(255,255,0);")
                self.button_start_stop.setIcon(self._icon_node_start)
            else:
                self.status.setStyleSheet("background:rgb(255,255,255);")
                self.status.setText('Unknown')
        
#End of file
