#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
# Copyright 2015 Airbus
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
import os
import sys
import threading

from actionlib_msgs.msg import GoalID
from std_msgs.msg import Bool

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from cobot_gui.alarm import Alarm
from cobot_gui.res import R

## @class EmergencyStopState
class EmergencyStopState:
    LOCKED   = True
    UNLOCKED = False

## @class EmergencyStopButton
class EmergencyStopButton(QPushButton):
    
    EMERGENCY_STOP_TOPIC_NAME = '/emergency_stop/state'
    
    def __init__(self, context):
        """! The constructor."""
        QPushButton.__init__(self)
        
        self._context = context
        self._context.addLanguageEventListner(self.onTranslate)
        self._context.addCloseEventListner(self.onDestroy)
        
        self.setCheckable(True)
        self.setFocusPolicy(Qt.NoFocus)
        self.setStyleSheet(R.values.styles.transparent_background)
        
        self.setIcon(R.getIconById("icon_pause"))
        self.setIconSize(QSize(80,80))
        
        self._button_state  = EmergencyStopState.UNLOCKED
        self._keep_running = True
        
        self.connect(self,SIGNAL('clicked(bool)'),self._trigger_button)
        
        self._estop_pub = rospy.Publisher(self.EMERGENCY_STOP_TOPIC_NAME,
                                          Bool, latch=True, queue_size=1)
        
        self._preempt_move_base_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        
        self._estop_pub_thread = threading.Thread(name='emergency_stop_publisher_loop',
                                                  target=self._emergency_stop_publisher_loop)
        self._estop_pub_thread.start()
        
    def _trigger_button(self, checked):
        """Called when user click on ermergency stop button.
        @param checked: Button status (True/False).
        @type checked: bool.
        """
        self._context.resquestEmergencyStop(checked)
        
        self._button_state = checked
        
        if checked == EmergencyStopState.LOCKED:
            lng = self._context.getLanguage()
            self._context.sendAlarm(Alarm.CRITICAL, R.values.strings.emergency_stop(lng))
            self.setIcon(R.getIconById("icon_play"))
        else:
            self.setIcon(R.getIconById("icon_pause"))
        
    def _emergency_stop_publisher_loop(self):
        """Loop to publish the emergency stop status."""
        
        r = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown() and self._keep_running:
            
            if self._button_state == EmergencyStopState.UNLOCKED:
                self._estop_pub.publish(Bool(True))
            else:
                self._preempt_move_base_pub.publish(GoalID())
                self._estop_pub.publish(Bool(False))
            
            r.sleep()
            
    def onTranslate(self, lng):
        pass
    
    def onDestroy(self):
        """Called when appli closes."""
        self._keep_running = False
        
##Unittest
if __name__ == "__main__":
    
    from cobot_gui.context import Context
    
    rospy.init_node('unittest_emergency_stop_2')
    
    a = QApplication(sys.argv)
    utt_appli = QMainWindow()
    context = Context(utt_appli)
    context.requestNewLanguage('fr')
    estop = EmergencyStopButton(context)
    estop.setIconSize(QSize(80,80))
    utt_appli.setCentralWidget(estop)
    utt_appli.show()
    a.exec_()
    estop.onDestroy()
    
    
#End of file
