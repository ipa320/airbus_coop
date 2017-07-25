#!/usr/bin/env python

## @package: robot_dashboard
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/04/2015

import os
import rospy

from actionlib_msgs.msg import GoalID
import arm_msgs

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from cobot_gui import DashboardPopup

from pma_dashboard.res import R

class PMADashboardPopup(DashboardPopup):
    
    def __init__(self, parent):
        """! Constructor.
        @param parent: parent.
        @type parent: C{QObject}.
        """
        DashboardPopup.__init__(self, parent)
        
    def onCreate(self, param):
        
        loadUi(R.layouts.pma1_popup, self)
        
        self.setRelativePosition(DashboardPopup.TopRight,
                                 DashboardPopup.BottomRight)
        
        self._preempt_move_base_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        
        self.preempt_button.setStyleSheet(R.values.styles.preempt)
        
        self.connect(self.preempt_button, SIGNAL("clicked()"), self._preempt_move_base)
        
    def _preempt_move_base(self):
        self._preempt_move_base_pub.publish(GoalID())
    
    def _update_pma1_status_(self, status):
        
        pass
    
    def _pma1_status_timeout_(self):
        pass
        
    def onDestroy(self):
        self._preempt_move_base_pub.unregister()
    
# my_unittest.shutdown()