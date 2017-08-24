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
import os
import sys
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiGui import QAgiSilderButton
from cobot_gui.alarm import Alarm
from cobot_gui.emergency import EmergencyStopState
from cobot_gui.res import R
from diagnostic_msgs.msg import DiagnosticStatus
from pyqt_agi_extend.QtAgiGui import QAgiMessageBox

## @class DiagnosticsStatus
## @brief Class for difine different control status.
class DiagnosticsStatus:
    
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3

    TOSTR = {OK : 'Ok',
             WARN : 'Warning',
             ERROR : 'Error',
             STALE : 'Stale'}

class DiagnosticsWidget(QPushButton):
    
    def __init__(self, context):
        """! The constructor."""
        QPushButton.__init__(self)

        self.state = 3
        self.msg = "No diagnostics data"
        self._context = context
        self._context.addCloseEventListner(self.onDestroy)
        self._diagnostics_toplevel_state_sub = rospy.Subscriber('diagnostics_toplevel_state', DiagnosticStatus, self.toplevel_state_callback)
        self.setIcon(R.getIconById("status_stale"))
        self.setIconSize(QSize(50,50))
        self.connect(self,SIGNAL('clicked(bool)'),self._trigger_button)

    def _trigger_button(self, checked):
        """Called when user click on ermergency stop button.
        @param checked: Button status (True/False).
        @type checked: bool.
        """
        msg_box = QAgiMessageBox()
        msg_box.setText(self.msg)
        #msg_box.setIcon(QAgiMessageBox.Critical)
        msg_box.setStandardButtons(QAgiMessageBox.Ok)
        msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
        msg_box.exec_()


    def onDestroy(self):
        """Called when appli closes."""
        self._keep_running = False

    def toplevel_state_callback(self, msg):
        self.state = msg.level
        self.msg = msg.message
        if self.state == 0:
          self.setIcon(R.getIconById("status_ok"))
          self.setIconSize(QSize(50,50))
        if self.state == 1 :
          self.setIcon(R.getIconById("status_warning"))
          self.setIconSize(QSize(50,50))
        if self.state == 2 :
          self.setIcon(R.getIconById("status_error"))
          self.setIconSize(QSize(50,50))
        if self.state == 3 :
          self.setIcon(R.getIconById("status_stale"))
          self.setIconSize(QSize(50,50))
        
if __name__ == "__main__":
    
    from cobot_gui.context import Context
    rospy.init_node('unittest_diagnostics_ui')
    a = QApplication(sys.argv)
    utt_appli = QMainWindow()
    context = Context(utt_appli)
    diag = DiagnosticsWidget(context)
    diag.setIconSize(QSize(80,80))
    utt_appli.setCentralWidget(diag)
    utt_appli.show()
    a.exec_()
    
#End of file
