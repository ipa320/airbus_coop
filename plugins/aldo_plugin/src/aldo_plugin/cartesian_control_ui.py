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
import tf

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from aldo_msgs.msg import CartesianGoals

from robot_axis_widget import RobotAxisWidget

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber
from pyqt_agi_extend.QtAgiGui import QAgiJoystick,\
                                     QAgiHJoystick

from aldo_plugin.res import R

class CartesianControlUi(QWidget):
    
    X = 0
    Y = 1
    Z = 2
    A = 3
    B = 4
    C = 5
    
    SPEED_THRESHOLD = 5.0
    PUBLISH_FREQUENCY = 10.0
    SPEED_INCREASED = 1.0
    
    DOF_NAME_LUT = ["x","y","z","a","b","c"]
    DOF_ID_LUT = [0,1,2,3,4,5]
    
    BASE_REGISTER = ["world","left_base_link", "right_base_link","left_tool0","right_tool0"]
    LEFT_TOOL_REGISTER = ["left_tool0"]
    RIGHT_TOOL_REGISTER = ["right_tool0"]
    
    def __init__(self):
        QWidget.__init__(self)
        
        loadUi(R.layouts.cart_control, self)
        self.base_combo.clear()
        self.left_tool_combo.clear()
        self.right_tool_combo.clear()
        self.base_combo.addItems(self.BASE_REGISTER)
        self.left_tool_combo.addItems(self.LEFT_TOOL_REGISTER)
        self.right_tool_combo.addItems(self.RIGHT_TOOL_REGISTER)
        self.setObjectName("cartControl")
        self.base_name = self.base_combo.currentText();
        self.tool_name = self.left_tool_combo.currentText();
        self.right_tool_combo.setEnabled(False)
        
        self.tf_listener = tf.TransformListener()
        
        self._position_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._position =[0.0,0.0,0.0,0.0,0.0,0.0]
        self.connect(self.change_frames, SIGNAL("clicked()"), self._slot_attach_new_tool)
        self.connect(self.switch_arm, SIGNAL("clicked()"), self._slot_switch_arm)
        
        self._update_state = QTimer(self)
        self.connect(self._update_state, SIGNAL("timeout()"), self._slot_update_state)
        self._reset_offset = QTimer(self)
        self.connect(self._reset_offset, SIGNAL("timeout()"), self._slot_reset_offset)
        self._pressed_timer = QTimer(self)
        self.connect(self._pressed_timer, SIGNAL("timeout()"), self._slot_publish_move)
        self._enable_movement = False
        self._arm_status = 0
        
        self._command_publisher = rospy.Publisher("/aldo/cartesian_cmd", CartesianGoals, queue_size=1)
        
        self._speed = 1.0
        self._dof_id = -1
        self._dof_step = 0
        
        for i in range(0,6):
            
            minus = getattr(self, "cart_%s_minus_button"%self.DOF_NAME_LUT[i])
            plus = getattr(self, "cart_%s_plus_button"%self.DOF_NAME_LUT[i])
            
            minus.setIcon(R.getIconById("ico_minus"))
            minus.setIconSize(QSize(40,40))
            
            plus.setIcon(R.getIconById("ico_plus"))
            plus.setIconSize(QSize(40,40))
            
            self.connect(minus,SIGNAL("pressed()"), self._slot_move)
            self.connect(minus,SIGNAL("released()"), self._slot_reset_offset)
             
            self.connect(plus,SIGNAL("pressed()"), self._slot_move)
            self.connect(plus,SIGNAL("released()"), self._slot_reset_offset)
    
    def _slot_switch_arm(self):  
        self.base_name = self.base_combo.currentText()
        if(self._arm_status == 0):
            self._arm_status = 1
            self.right_tool_combo.setEnabled(True)
            self.left_tool_combo.setEnabled(False)
        elif(self._arm_status == 1):
            self._arm_status = 0
            self.right_tool_combo.setEnabled(False)
            self.left_tool_combo.setEnabled(True)
        else:
            return
        self._slot_attach_new_tool()

        
    def _slot_attach_new_tool(self):
        
        self.base_name = self.base_combo.currentText()
        if(self._arm_status == 0):
            self.tool_name = self.left_tool_combo.currentText()
        elif(self._arm_status == 1):
            self.tool_name = self.right_tool_combo.currentText()
        else:
            return
        
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
        
    def _slot_move(self):
        self._reset_offset.stop()
        self._pressed_timer.start(1000.0/self.PUBLISH_FREQUENCY)
        self._dof_id, self._dof_step = self._get_cart_id_clicked(self.sender())
        
    
    def _slot_publish_move(self):
        self._position_offset[self._dof_id] = self._position_offset[self._dof_id] + self._dof_step*self._speed
        if(self._speed < self.SPEED_THRESHOLD):
            self._speed = self._speed + self.SPEED_INCREASED*(1/self.PUBLISH_FREQUENCY)
        
            
        
        msg = CartesianGoals()
        msg.header.stamp = rospy.Time.now()
        if(self._arm_status == 0):
            msg.left_arm.use.data = True
            msg.left_arm.base_frame.data = "/"+self.base_name
            msg.left_arm.tool_frame.data = "/"+self.tool_name    
            msg.left_arm.pose.position.x =self._position_offset[0]/1000.0
            msg.left_arm.pose.position.y =self._position_offset[1]/1000.0
            msg.left_arm.pose.position.z =self._position_offset[2]/1000.0
            quat = tf.transformations.quaternion_from_euler(math.radians(self._position_offset[3]), math.radians(self._position_offset[4]), math.radians(self._position_offset[5]), "rzyx")
            msg.left_arm.pose.orientation.x =quat[0]
            msg.left_arm.pose.orientation.y =quat[1]
            msg.left_arm.pose.orientation.z =quat[2]
            msg.left_arm.pose.orientation.w =quat[3]
            msg.right_arm.use.data = False
            msg.right_arm.base_frame.data = "/"
            msg.right_arm.tool_frame.data = "/"
            msg.right_arm.pose.position.x =0.0
            msg.right_arm.pose.position.y =0.0
            msg.right_arm.pose.position.z =0.0
            msg.right_arm.pose.orientation.x =0.0
            msg.right_arm.pose.orientation.y =0.0
            msg.right_arm.pose.orientation.z =0.0
            msg.right_arm.pose.orientation.w =1.0
        if(self._arm_status == 1):
            msg.right_arm.use.data = True
            msg.right_arm.base_frame.data = "/"+self.base_name
            msg.right_arm.tool_frame.data= "/"+self.tool_name
            msg.right_arm.pose.position.x =self._position_offset[0]/1000.0
            msg.right_arm.pose.position.y =self._position_offset[1]/1000.0
            msg.right_arm.pose.position.z =self._position_offset[2]/1000.0
            quat = tf.transformations.quaternion_from_euler(math.radians(self._position_offset[3]), math.radians(self._position_offset[4]), math.radians(self._position_offset[5]), "rzyx")
            msg.right_arm.pose.orientation.x =quat[0]
            msg.right_arm.pose.orientation.y =quat[1]
            msg.right_arm.pose.orientation.z =quat[2]
            msg.right_arm.pose.orientation.w =quat[3]
            msg.left_arm.use.data = False
            msg.left_arm.base_frame = "/"
            msg.left_arm.tool_frame = "/"
            msg.left_arm.pose.position.x =0.0
            msg.left_arm.pose.position.y =0.0
            msg.left_arm.pose.position.z =0.0
            msg.left_arm.pose.orientation.x =0.0
            msg.left_arm.pose.orientation.y =0.0
            msg.left_arm.pose.orientation.z =0.0
            msg.left_arm.pose.orientation.w =1.0

        self._command_publisher.publish(msg)
    
    def _slot_reset_offset(self):
        self._speed = 1.0
        self._pressed_timer.stop()
        self._reset_offset.start(150)
        self._position_offset = self._position
        
    def enable_movement(self):
        for i in range(0,6):          
            minus = getattr(self, "cart_%s_minus_button"%self.DOF_NAME_LUT[i])
            plus = getattr(self, "cart_%s_plus_button"%self.DOF_NAME_LUT[i])
            minus.setEnabled(True)
            plus.setEnabled(True)
    
    def disable_movement(self):
        for i in range(0,6):
            minus = getattr(self, "cart_%s_minus_button"%self.DOF_NAME_LUT[i])
            plus = getattr(self, "cart_%s_plus_button"%self.DOF_NAME_LUT[i])
            minus.setEnabled(False)
            plus.setEnabled(False)
        

    def _slot_update_state(self):
        
        try:
            (trans,rot) = self.tf_listener.lookupTransform("/"+self.base_name, "/"+self.tool_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if(self._enable_movement):
                self.disable_movement()
                self._enable_movement = False
            return
        
        if(self._enable_movement == False):
            self._enable_movement = True
            self.enable_movement()
            
        euler = tf.transformations.euler_from_quaternion(rot, "rzyx")
            
        self._position = [trans[0]*1000.0, trans[1]*1000.0, trans[2]*1000.0, math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2])]
        
        self.current_pos_x.setText("%.4f"%(self._position[0]/1000.0))
        self.current_pos_y.setText("%.4f"%(self._position[1]/1000.0))
        self.current_pos_z.setText("%.4f"%(self._position[2]/1000.0))
        
        self.current_rot_a.setText("%.4f"%self._position[3])
        self.current_rot_b.setText("%.4f"%self._position[4])
        self.current_rot_c.setText("%.4f"%self._position[5])
        
        
    def subscribe(self):
        self._speed = 1.0
        self._update_state.start(100)
        self._reset_offset.start(150)
        self.disable_movement()
        self._enable_movement = False
        
    def unregister(self):
        self._update_state.stop()
        self._reset_offset.stop()
        self._pressed_timer.stop()

        
if __name__ == "__main__":
    
    from python_qt_binding.QtGui import QApplication
    import sys
    import rospy
    
    rospy.init_node('aldo_plugin_test_2')
    
    app = QApplication(sys.argv)
    
    MainWindow = CartesianControlUi()
    MainWindow.subscribe()
    MainWindow.show()
    
    app.exec_()
    
    MainWindow.unregister()
        
