#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################


import rospy

from geometry_msgs.msg import Twist

from python_qt_binding.QtGui import *
from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix

from cobot_gui import Plugin, ControlMode
from omni_gui_v5 import RemoteWidget

from omni_gui.res import R

import rviz

class PluginOmniV2(Plugin):
    
    def __init__(self, context):
        Plugin.__init__(self, context)
        
    def onCreate(self, param):
        
        yaml          = get_pkg_dir_from_prefix(param.getParam('yaml', "${omni_gui}/resources/yaml/sara.rviz"))
        linear_speed  = float(param.getParam('linear_speed', 0.5))
        angular_speed = float(param.getParam('angular_speed', 0.5))
        
        grid_layout = QGridLayout(self)
        grid_layout.setSpacing(5)
        grid_layout.setContentsMargins(10,10,10,10)
        grid_layout.setHorizontalSpacing(5)
        grid_layout.setVerticalSpacing(5)
        
        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.setSplashPath( "" )
        self.rviz_frame.initialize()
         
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, yaml)
        self.rviz_frame.load( config )
        self.rviz_frame.setMenuBar( None )
        self.rviz_frame.setHideButtonVisibility( False )
         
        self._rviz = self.rviz_frame.centralWidget()
         
        self._rw = QWidget()
        self._rw.setStyleSheet(R.values.styles.rviz)
        self.rwl = QGridLayout(self._rw)
        self.rwl.setContentsMargins(5,5,5,5)
        self.rwl.addWidget(self._rviz, 0, 0)
         
        spacer = QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding)
        grid_layout.addItem(spacer, 0, 1)
         
        grid_layout.addWidget(self._rw, 0, 0)
        
        self._remote = RemoteWidget()
        self._remote.setLinearSpeed(linear_speed)
        self._remote.setAngularSpeed(angular_speed)
        
        grid_layout.addWidget(self._remote, 1, 0)
        
    def onPause(self):
        self._remote.stop()
    
    def onResume(self):
        self._remote.start()
    
    def onControlModeChanged(self, mode):
        
        if mode == ControlMode.AUTOMATIC:
            self.setEnabled(False)
        else:
            self.setEnabled(True)
    
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        self._remote.requestPreemptMotion()
    
    def onDestroy(self):
        self._remote.stop()
    
#End of file
