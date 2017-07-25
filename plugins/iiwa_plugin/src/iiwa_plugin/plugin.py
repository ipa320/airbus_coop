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
import os

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from industrial_msgs.msg import RobotStatus

from iiwa_driver import IIWAServoMotion
from joint_control_ui import JointControlUi
from cartesian_control_ui import CartesianControlUi
from io_control_ui import IOControlUi

from robot import RobotUi
from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from cobot_gui import plugin, ControlMode, Alarm

from iiwa_plugin.res import R

class IIWAConnectionThread(QThread):
    
    def __init__(self, parent, robot, timeout = rospy.Duration(2)):
        QThread.__init__(self, parent)
        
        self._robot = robot
        self._timeout = timeout
        
    def run(self):
        res = self._robot.waitForRobotConnection(self._timeout)
        self.emit(SIGNAL("connectionStatus"), res)

class IIWAPlugin(plugin.Plugin):
    
    INDEX_TAB_FROM_JOINT_CTRL = 0
    INDEX_TAB_FROM_CART_CTRL  = 1
    INDEX_TAB_FROM_IO_CTRL    = 2
    
    def __init__(self, context):
        plugin.Plugin.__init__(self, context)
        
    def onCreate(self, param):
        
        loadUi(R.layouts.mainwidow, self)
        
        self._robot_ui = RobotUi(self)
        robot_widget = self._robot_ui.get_widget()
        self.robot_view_area.setWidget(robot_widget)
        
        self._servo_motion = IIWAServoMotion()
        
        self.joint_control = JointControlUi(self._servo_motion)
        
        self.motion_tab.addTab(self.joint_control, "Joint")
        
        self.cart_control = CartesianControlUi(self._servo_motion)
        
        self.motion_tab.addTab(self.cart_control,"Cartesian")
        
        self.io_control = IOControlUi()
        
        self.motion_tab.addTab(self.io_control,"I/O")
        
        self.connect(self.motion_tab, SIGNAL("currentChanged(int)"),
                     self.slot_switch_motion_type)
        
        self.connect(self.gravity_motion_button,
                     SIGNAL("pressed()"),
                     self._servo_motion.gravity)
         
        self.connect(self.gravity_motion_button,
                     SIGNAL("released()"),
                     self._servo_motion.stopMotion)
        
        self._establish_connection = IIWAConnectionThread(self, self._servo_motion)
        self.connect(self._establish_connection,
                     SIGNAL("connectionStatus"),
                     self.connectionEstablished)
        
        self._last_error_code = 0
        self._robot_status_comm = QAgiSubscriber(self,
                                                '/iiwa/status',
                                                RobotStatus,
                                                self._slot_robot_status,
                                                max_rate = 30)
        
    def _slot_robot_status(self, status):
        
        if status.error_code < 0 and status.error_code != self._last_error_code:
            self.joint_control.stop_motion()
            self.cart_control.stop_motion()
            
        self._last_error_code = status.error_code
        
    def slot_switch_motion_type(self, index):
        
        self.joint_control.unregister()
        self.cart_control.unregister()
        self.io_control.unregister()
         
        if index == self.INDEX_TAB_FROM_JOINT_CTRL:
            self.joint_control.subscribe()
        elif index == self.INDEX_TAB_FROM_CART_CTRL:
            self.cart_control.subscribe()
        elif index == self.INDEX_TAB_FROM_IO_CTRL:
            self.io_control.subscribe()
            
    def onStart(self):
        self.onResume()
        
    def onPause(self):
        self._servo_motion.shutdown()
        self.joint_control.unregister()
        self.cart_control.unregister()
        self.io_control.unregister()
        self._robot_status_comm.unregister()
        
    def connectionEstablished(self, state):
        
        if state is False:
            self.sendAlarm(Alarm.WARNING, "IIWA connection not established !")
        else:
            self._robot_status_comm.subscribe()
            self.joint_control.subscribe()
            self.cart_control.subscribe()
            self.io_control.subscribe()
        
    def onResume(self):
        self._establish_connection.start()
        
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
        self.onPause()
        
    def onDestroy(self):
        self.onPause()
    
    
if __name__ == "__main__":
    
    import sys
    
    rospy.init_node("iiwa_plugin_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("iiwa_plugin", IIWAPlugin)
    window.setWindowTitle("IIWA Controller Interface")
    window.show()
    a.exec_()
    
#End of file

