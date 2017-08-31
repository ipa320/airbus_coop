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
import threading
from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi
from cobot_gui.res import R
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from pyqt_agi_extend.QtAgiGui import QAgiPopup
from rqt_robot_monitor.status_item import StatusItem
import rqt_robot_monitor.util_robot_monitor as util

## @class DiagnosticsStatus
## @brief Class for difine different control status.

    
#OK = 0
#WARN = 1
#ERROR = 2
#STALE = 3


class DiagnosticsWidget(QPushButton):
    
    DIAGNOSTICS_TOPLEVEL_TOPIC_NAME = rospy.get_param('diagnostics_toplevel_topic_name','/diagnostics_toplevel_state')

    def __init__(self, context):
        """! The constructor."""
        QPushButton.__init__(self)
        self._context = context
        self._context.addCloseEventListner(self.onDestroy)
        self.state = "status_stale"
        self.msg = "No diagnostic messages received"
        self.setToolTip(self.msg)
        self.setIcon(R.getIconById(self.state))
        self.setIconSize(QSize(40,40))
        self.connect(self,SIGNAL('clicked(bool)'),self._trigger_button)
        self._diagnostics_toplevel_state_sub = rospy.Subscriber(self.DIAGNOSTICS_TOPLEVEL_TOPIC_NAME , DiagnosticStatus, self.toplevel_state_callback)

    def _trigger_button(self, checked):
        popup = DiagnosticsPopup(self)
        popup.show_()

    def toplevel_state_callback(self, msg):
        self.state = msg.level
        if msg.level == 0:
          self.state= "status_ok"
          self.msg = "OK"
        if msg.level == 1 :
          self.state= "status_warning"
          self.msg = "WARNING"
        if msg.level == 2 :
          self.state= "status_error"
          self.msg = "ERROR"
        if msg.level == 3 :
          self.state= "status_stale"
          self.msg = "STALE"

        self.setToolTip(self.msg)
        self.setIcon(R.getIconById(self.state))
        self.setIconSize(QSize(40,40))

    def onDestroy(self):
        """Called when appli closes."""
        self._keep_running = False

class DiagnosticsPopup(QAgiPopup):

    def __init__(self, parent):
        """! The constructor."""
        QAgiPopup.__init__(self, parent)
        self._parent = parent
        self.setRelativePosition(QAgiPopup.TopRight, QAgiPopup.BottomRight)
        loadUi(R.layouts.diagnostics_popup, self)
        DIAGNOSTICS_TOPIC_NAME = rospy.get_param('diagnostics_topic_name','/diagnostics_agg')
        self._diagnostics_agg_sub = rospy.Subscriber(DIAGNOSTICS_TOPIC_NAME, DiagnosticArray, self.message_cb)
        self._inspectors = {}
        self._current_msg = None
        palette = self.tree_all_devices.palette()
        self._original_base_color = palette.base().color()
        self._original_alt_base_color = palette.alternateBase().color()
        self._tree = StatusItem(self.tree_all_devices.invisibleRootItem())
        self.adjustSize()

    def message_cb(self,msg):
        """ DiagnosticArray message callback """
        for status in msg.status:
            path = status.name.split('/')
            if path[0] == '':
                path = path[1:]
            tmp_tree = self._tree
            for p in path:
                tmp_tree = tmp_tree[p]
            tmp_tree.update(status, util.get_resource_name(status.name))
        self._tree.prune()
        self.tree_all_devices.resizeColumnToContents(0)
        self.adjustSize()

if __name__ == "__main__":
    from cobot_gui.context import Context
    rospy.init_node('unittest_diagnostics_ui')
    a = QApplication(sys.argv)
    utt_appli = QMainWindow()
    context = Context(utt_appli)
    diag = DiagnosticsWidget(context)
    diag.setIconSize(QSize(40,40))
    utt_appli.setCentralWidget(diag)
    utt_appli.show()
    a.exec_()
    diag.onDestroy()
    
#End of file
