#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : omni_gui_v5.py
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
from geometry_msgs.msg import Twist

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiGui import QAgiJoystick,\
                                     QAgiHJoystick
                                     
from omni_gui.res import R

import rviz
import tf

class OdomListener(QThread):
    
    def __init__(self):
        QThread.__init__(self)
        
        self.listener = tf.TransformListener()
        self._is_running = True
        
    def stop(self):
        self._is_running = False
        
    def run(self):
        
        r = rospy.Rate(1)
        self._is_running = True
        
        while not rospy.is_shutdown() and self._is_running:
            try:
                trans, rot = self.listener.lookupTransform('/map',
                                                           '/base_footprint',
                                                           rospy.Time(0))
                self.emit(SIGNAL('updateOdom'),trans, rot)
            except:
                pass
            r.sleep()

class NavigationDashboard(QLabel):
    
    def __init__(self):
        QLabel.__init__(self)
        
        self.setFixedSize(250,150)
        
        loadUi(R.layouts.navigation_dashbord, self)
        
        self.setStyleSheet(R.values.styles.dashboard_background)
        
        self.trans_label.setStyleSheet(R.values.styles.dashboard_text)
        self.trans_x.setStyleSheet(R.values.styles.dashboard_text)
        self.trans_y.setStyleSheet(R.values.styles.dashboard_text)
        self.trans_x_label.setStyleSheet(R.values.styles.dashboard_text)
        self.trans_y_label.setStyleSheet(R.values.styles.dashboard_text)
        
        self.rot_label.setStyleSheet(R.values.styles.dashboard_text)
        self.rot_z.setStyleSheet(R.values.styles.dashboard_text)
        self.rot_w.setStyleSheet(R.values.styles.dashboard_text)
        self.rot_z_label.setStyleSheet(R.values.styles.dashboard_text)
        self.rot_w_label.setStyleSheet(R.values.styles.dashboard_text)
        
        self._odom_listener_thread = OdomListener()
        
        self.connect(self._odom_listener_thread,
                     SIGNAL('updateOdom'),
                     self.update_odom)
        
    def start(self):
        self._odom_listener_thread.start()
    
    def stop(self):
        self._odom_listener_thread.stop()
        
    def update_odom(self, trans, rot):
        self.trans_x.setText('%.3f'%trans[0])
        self.trans_y.setText('%.3f'%trans[1])
        self.rot_z.setText('%.4f'%rot[2])
        self.rot_w.setText('%.4f'%rot[3])
        
    def resizeEvent(self, event):
        self.rot_label.setPixmap(R.getPixmapById("rotation").scaled(
                                                  self.rot_label.width(),
                                                  self.rot_label.height()))
        
        self.trans_label.setPixmap(R.getPixmapById("localization").scaled(
                                                  self.rot_label.width(),
                                                  self.rot_label.height()))

class RemoteWidget(QWidget):
    
    ## Max linear speed allowed (m/s)
    MAX_LIN_SPEED = 0.45 #0.250
    
    ## Max angular speed allowed (rad/s)
    MAX_ANG_SPEED = 0.500
    
    def __init__(self):
        QWidget.__init__(self)
        
        layout = QHBoxLayout(self)
        layout.setSpacing(5)
        
        self._lin_speed = self.MAX_LIN_SPEED
        self._ang_speed = self.MAX_ANG_SPEED
        
        self._twist = Twist()
        self._twist_pub = None
        
        self._joystick_rot = QAgiHJoystick()
        self._joystick_rot.setHandleSize(QSize(50,50))
        self._joystick_rot.setFixedSize(QSize(150,150))
        
        layout.addWidget(self._joystick_rot)
        
        spacer = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)
        layout.addItem(spacer)
        
        self._dashboard_nav = NavigationDashboard()
        layout.addWidget(self._dashboard_nav)
        
        spacer = QSpacerItem(0, 0, QSizePolicy.Expanding, QSizePolicy.Minimum)
        layout.addItem(spacer)
        
        self._joystick_trans = QAgiJoystick()
        self._joystick_trans.setHandleSize(QSize(50,50))
        self._joystick_trans.setFixedSize(QSize(150,150))
        
        layout.addWidget(self._joystick_trans)
        
        self.connect(self._joystick_rot, SIGNAL("valuesChanged"), self._slot_update_rot)
        self.connect(self._joystick_trans, SIGNAL("valuesChanged"), self._slot_update_trans)
        
    def setLinearSpeed(self, speed):
        self._lin_speed = speed
        
    def setAngularSpeed(self, speed):
        self._ang_speed = speed
        
    def _slot_update_trans(self, translation):
        self._twist.linear.x = self._lin_speed * translation[1]
        self._twist.linear.y = -(self._lin_speed* translation[0])
        
        self._twist_pub.publish(self._twist)
        
    def _slot_update_rot(self, rotation):
        self._twist.angular.z = -(self._ang_speed * rotation[0])
        
        self._twist_pub.publish(self._twist)
        
    def start(self):
        self._twist_pub = rospy.Publisher('cmd_vel', Twist, latch = True, queue_size=1)
        self._dashboard_nav.start()
    
    def stop(self):
        if self._twist_pub is not None:
            self._twist_pub.unregister()
            self._twist_pub = None
        self._dashboard_nav.stop()
        
    def requestPreemptMotion(self):
        if self._twist_pub is not None:
            self._twist_pub.publish(Twist())
    
#End of file

