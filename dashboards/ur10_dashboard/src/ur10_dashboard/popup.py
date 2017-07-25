#!/usr/bin/env python

## @package: robot_dashboard
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/04/2015

import os
import rospy

from cobot_gui import DashboardPopup

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from ur10_dashboard.res import R

class UR10DashboardPopup(DashboardPopup):
    
    def __init__(self, parent):
        DashboardPopup.__init__(self,
                                parent,
                                DashboardPopup.TopRight,
                                DashboardPopup.BottomRight)
        
    def onCreate(self, param):
        pass
        
    def onDestroy(self):
        pass
    
# my_unittest.shutdown()
