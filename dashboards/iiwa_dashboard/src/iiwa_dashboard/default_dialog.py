#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : accounts_ui.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from industrial_msgs.msg import RobotStatus
from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from arm_msgs.srv import AcquitError

from iiwa_dashboard.res import R

## @package: user_account
##
## @version 2.2
## @author  Matignon Martin
## @date    Last modified 22/05/2014

## @class LoginDialog
## @brief Login user interface.
class RobotDefaultHandleDialog(QDialog):
    
    DEFAULT_PROXY_SERVER_NAME = "/iiwa/acquit_default"
    
    def __init__(self, parent, default):
        """! The constructor."""
        
        QDialog.__init__(self, parent, Qt.FramelessWindowHint)
        
        self._parent = parent
        
        # Get path to UI file which is a sibling of this file
        loadUi(R.layouts.defaults, self)
        
        self.setAttribute(Qt.WA_StyledBackground)
        self.setModal(True)
        self.setStyleSheet(R.values.styles.default_dialog)
        
        self._lng = parent.getContext().getLanguage()
        
        self.icon.setPixmap(R.getPixmapById("icon_robot_in_default"))
        self.what.setText(R.values.strings.robot_in_error(self._lng)+"\nError : %i"%default)
        self.acquit_button.setStyleSheet(R.values.styles.acquit_button)
        
        self._default = default
        self.connect(self.acquit_button,SIGNAL("clicked()"),self._slot_acquit_defaulf)
        
        self._robot_status_sub = QAgiSubscriber(self,
                                              '/iiwa/status',
                                              RobotStatus,
                                              self._slot_robot_status,
                                              timeout  = rospy.Duration(1),
                                              max_rate = 3)
        self.adjustSize()
        
    def _slot_robot_status(self,status):
        if status.error_code >= 0:
            self.close()
        
    def _slot_acquit_defaulf(self):
        
        self.acquit_button.setEnabled(False)
        try:
            rospy.wait_for_service(self.DEFAULT_PROXY_SERVER_NAME, 1)
            proxy = rospy.ServiceProxy(self.DEFAULT_PROXY_SERVER_NAME, AcquitError)
            print proxy(self._default)
        except Exception as ex:
            self._parent.getContext().getLogger().critical(R.values.strings.acquit_timeout_exceeded(self._lng))
        
        self.close()
        
    def closeEvent(self, event):
        self._robot_status_sub.shutdown()
        
if __name__ == "__main__":
    
    rospy.init_node('unittest_default_ui')
    
    import sys
    
    a = QApplication(sys.argv)
    
    utt_appli = QMainWindow()
    
    utt_appli.setCentralWidget(RobotDefaultHandleDialog(None, -3))
    
    utt_appli.show()
    a.exec_()
    
#End of file