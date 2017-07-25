#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : setup.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
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
        
        print 'coucou'
        
        rospy.loginfo('%s::_launch_node()'%self.launch_name.text())
        
        subprocess.Popen(['roslaunch',
                          'node_launchers',
                          self.launch_name.text()])
        
#End of file
