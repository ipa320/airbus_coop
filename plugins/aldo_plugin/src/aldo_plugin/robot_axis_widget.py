#!/usr/bin/env python

import rospy
import os
import math
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from aldo_plugin.res import R
# Note :
# Optimization : create LUT for function afine

class RobotAxisWidget(QWidget):
    
#     RESOURCES_DIR = get_pkg_dir('plugin_robot_arm_control')+"/resources"
    
    def __init__(self, angle_min, angle_max, torque_max = 110):
        QWidget.__init__(self)
        
        loadUi(R.layouts.robot_axis,self)
        
        self.px_min    = 0.
        self.px_max    = float(self.joint_angle_background.width() - self.joint_angle_handle.width())

        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        self.angle_px   = (self.px_max - self.px_min) / (self.angle_max - self.angle_min)
        
        self.torque_min = float(-torque_max)
        self.torque_max = float(torque_max)
        self.torque_a   = (self.px_max - self.px_min) / (self.torque_max - self.torque_min)
        self.torque_b   = self.px_max / 2.
        
        self.current_angle  = 0.
        self.current_torque = 0.
        
    def setJointAngle(self, angle):
        
        self.current_angle = angle
        
        if float(self.current_angle) >= 0.:
            self.joint_angle_handle.setText("+%.1f"%float(self.current_angle))
        else:
            self.joint_angle_handle.setText("%.1f"%float(self.current_angle))
        
        
        self.joint_angle_handle.move(int(self.angle_px*(self.current_angle - self.angle_min)), 0)
        
        if angle <= self.angle_min+5 or angle >= self.angle_max-5:
            self.joint_angle_handle.setStyleSheet(R.values.styles.out_joint_angle)
        else:
            self.joint_angle_handle.setStyleSheet(R.values.styles.joint_angle_ok)
            
    def setJointTorque(self, torque):
        
        self.current_torque = torque
        
        value = int(self.torque_a * self.current_torque + self.torque_b)
        
        if self.current_torque > 2.:
            self.joint_torque_handle.setGeometry(QRect(self.torque_b, 0, value - self.torque_b, self.joint_torque_background.height()))
        elif self.current_torque < -2.:
            self.joint_torque_handle.setGeometry(QRect(value, 0, self.torque_b - value, self.joint_torque_background.height()))
        else:
            self.joint_torque_handle.setGeometry(QRect(self.torque_b-2, 0, 4, self.joint_torque_background.height()))
    
    def resizeEvent(self, event):
        
        
        
        self.joint_angle_handle.setFixedSize(QSize(100, self.joint_angle_background.height()))
        self.px_max   = float(self.joint_angle_background.width() - self.joint_angle_handle.width())
        self.angle_px = (self.px_max - self.px_min) / (self.angle_max - self.angle_min)
        self.setJointAngle(self.current_angle)
        
        self.torque_a = (self.px_max - self.px_min) / (self.torque_max - self.torque_min)
        self.torque_b = self.px_max / 2.
        
        self.setJointTorque(self.current_torque)
        
        
if __name__ == "__main__":
    
    from python_qt_binding.QtGui import QApplication
    import sys
    import rospy
    
    rospy.init_node('plugin_robot_arm_control')
    
    app = QApplication(sys.argv)
    
    from pyqt_agi_extend.QtAgiCore import loadRsc
    
    rsc = loadRsc("iiwa_plugin")
    
    MainWindow = RobotAxisWidget(rsc, -170, 170)
    MainWindow.resize(QSize(500,40))
    MainWindow.setJointAngle(0.0)
    MainWindow.show()
    
    app.exec_()
    
#End of file

