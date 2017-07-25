#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : cartesian_move.py
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
import math
import threading
import copy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from industrial_msgs.msg import RobotStatus
from geometry_msgs.msg import Pose
from arm_msgs.msg import CartesianTrajectoryPoint

from robot_axis_widget import RobotAxisWidget

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber
from pyqt_agi_extend.QtAgiGui import QAgiJoystick,\
                                     QAgiHJoystick

from iiwa_plugin.res import R

class CartesianControlUi(QWidget):
    
    X = 0
    Y = 1
    Z = 2
    A = 3
    B = 4
    C = 5
    
    DOF_NAME_LUT = ["x","y","z","a","b","c"]
    DOF_ID_LUT = [0,1,2,3,4,5]
    
    BASE_REGISTER = {"Robot":0,"Word":1}
    TOOL_REGISTER = {"Empty":0,"Stamp":1,"uEpsilon":2}
    TCP_REGISTER  = {"TCP":0}
    
    def __init__(self, servo_motion):
        QWidget.__init__(self)
        
        loadUi(R.layouts.cart_control, self)
        
        self.setObjectName("cartControl")
        
        self._servo_motion = servo_motion
        
#         self.connect(self.tool_name_combo, SIGNAL("currentIndexChanged(int)"), self._slot_tool_changed)
#         self.connect(self.tcp_name_combo, SIGNAL("currentIndexChanged(int)"), self._slot_tool_changed)
        
        self._refresh_rate = 0
        self._position_offset = [0, 0, 0, 0, 0, 0]
        
        self._pose_states = Pose()
        self._pose_states_comm = QAgiSubscriber(self,
                                                '/iiwa/pose_cart',
                                                Pose,
                                                self._slot_update_current_pose_states,
                                                max_rate = 30)
        
        
        self._refresh_frames_rate = 0
        self._frames_state_comm = QAgiSubscriber(self,
                                                'iiwa/status',
                                                RobotStatus,
                                                self._slot_update_frames,
                                                max_rate = 10)
        
        self.attach_tool_button.setIcon(R.getIconById("attcah_tool"))
        self.attach_tool_button.setIconSize(QSize(80,80))
        
        self.connect(self.attach_tool_button, SIGNAL("clicked()"), self._slot_attach_new_tool)
        
        self._goal = CartesianTrajectoryPoint()
        self._goal.velocity = 100
        self._goal.forces = [20,20,20]
        self._goal.stiffness = [3000,3000,3000,300,300]
        self._trigger_pub = QTimer(self)
        self.connect(self._trigger_pub, SIGNAL("timeout()"), self._publish_cart_command)
        
        
        self._dof_id = -1
        self._dof_step = 0
        
        for i in range(0,6):
            
            minus = getattr(self, "cart_%s_minus_button"%self.DOF_NAME_LUT[i])
            plus = getattr(self, "cart_%s_plus_button"%self.DOF_NAME_LUT[i])
            
            minus.setIcon(R.getIconById("ico_minus"))
            minus.setIconSize(QSize(40,40))
            
            plus.setIcon(R.getIconById("ico_plus"))
            plus.setIconSize(QSize(40,40))
            
            self.connect(minus,SIGNAL("pressed()"), self._slot_start_motion)
            self.connect(minus,SIGNAL("released()"), self.stop_motion)
             
            self.connect(plus,SIGNAL("pressed()"), self._slot_start_motion)
            self.connect(plus,SIGNAL("released()"), self.stop_motion)
        
    def _slot_update_frames(self, iiwa):
        
        if self._refresh_frames_rate%1000 == 1:
            
            self._refresh_frames_rate = 0
            
            robot_frames = iiwa.header.frame_id.split('/')[1:]
            
            if len(robot_frames) == 3:
                self.current_base_frame.setText(robot_frames[0])
                self.current_tool_frame.setText(robot_frames[1])
                self.current_tcp_frame.setText(robot_frames[2])
                
        self._refresh_frames_rate += 1
        
    def _slot_attach_new_tool(self):
        
        base_name = self.base_name_combo.currentText()
        self._servo_motion.setBaseFrame("/%s"%base_name)
        
        tool_name = self.tool_name_combo.currentText()
        tcp_name = self.tcp_name_combo.currentText()
        
        self._servo_motion.attachTool(tool_name,tcp_name)
        
    def _get_cart_id_clicked(self, sender_id):
        
        self._curr_sender_id = sender_id.objectName()
        
        for i in range(0,6):
            
            minus = getattr(self, "cart_%s_minus_button"%self.DOF_NAME_LUT[i])
            plus  = getattr(self, "cart_%s_plus_button"%self.DOF_NAME_LUT[i])
            
            if plus.objectName() == sender_id.objectName():
                return (i, 1)
            elif minus.objectName() == sender_id.objectName():
                return (i, -1)
            else:
                continue
            
        return (-1, 0)
        
    def _slot_start_motion(self):
        self._dof_id, self._dof_step = self._get_cart_id_clicked(self.sender())
        self._trigger_pub.start(100)
        
    def stop_motion(self):
        self._trigger_pub.stop()
        self._servo_motion.stopMotion();

    def _slot_update_current_pose_states(self, pose):
        
        self._pose_states = pose
        
        if (self._refresh_rate % 1000) == 1:
            self._refresh_rate = 0
            self.current_pos_x.setText("%.4f"%self._pose_states.position.x)
            self.current_pos_y.setText("%.4f"%self._pose_states.position.y)
            self.current_pos_z.setText("%.4f"%self._pose_states.position.z)
            
            self.current_rot_a.setText("%.4f"%math.degrees(self._pose_states.orientation.x))
            self.current_rot_b.setText("%.4f"%math.degrees(self._pose_states.orientation.y))
            self.current_rot_c.setText("%.4f"%math.degrees(self._pose_states.orientation.z))
            
        self._refresh_rate+=1
        
    def _publish_cart_command(self):
        
        self._goal.positions = [self._pose_states.position.x,
                                self._pose_states.position.y,
                                self._pose_states.position.z,
                                self._pose_states.orientation.x,
                                self._pose_states.orientation.y,
                                self._pose_states.orientation.z]
        
        if self._dof_id >= self.X and self._dof_id <= self.Z:
            
            self._goal.positions[self._dof_id] += 50*self._dof_step
            
        elif self._dof_id >= self.A and self._dof_id <= self.C:
            
            self._goal.positions[self._dof_id] += math.radians(5)*self._dof_step
        
        self._servo_motion.moveAsync(self._goal)
        
    def subscribe(self):
        self._frames_state_comm.subscribe()
        self._pose_states_comm.subscribe()
        
    def unregister(self):
        self._frames_state_comm.shutdown()
        self._pose_states_comm.unregister()
        
    def resizeEvent(self, event):
        self.label_frames.setPixmap(R.getPixmapById("iiwa_frame").scaled(
                                                self.label_frames.width()-5,
                                                self.label_frames.height()-5,
                                                Qt.KeepAspectRatio,
                                                Qt.SmoothTransformation
                                    ))
        
