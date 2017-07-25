#!/usr/bin/env python

import rospy
import os
import math
import threading

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from sensor_msgs.msg import JointState
from aldo_msgs.msg import ArticularGoals

from robot_axis_widget import RobotAxisWidget

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from aldo_plugin.res import R

class LeftJointControlUi(QWidget):
    
    AXIS_NAME = ["left_shoulder_pan_joint","left_shoulder_lift_joint","left_elbow_joint", 
                  "left_wrist_1_joint","left_wrist_2_joint", "left_wrist_3_joint"]
    
    AXIS_MIN_LIMITS = [math.degrees(-1*math.pi),
                       math.degrees(-1*math.pi),
                       math.degrees(-1*math.pi),
                       math.degrees(-2*math.pi),
                       math.degrees(-1*math.pi),
                       math.degrees(-1*math.pi)]
    AXIS_MAX_LIMITS = [math.degrees(math.pi/4.0),
                       math.degrees(1.0),
                       math.degrees(math.pi),
                       math.degrees(0.0),
                       math.degrees(math.pi),
                       math.degrees(math.pi)]
    
    SPEED_THRESHOLD = 5.0
    PUBLISH_FREQUENCY = 10.0
    SPEED_INCREASED = 1.0
    
    def __init__(self):
        QWidget.__init__(self)
        
        loadUi(R.layouts.joint_control, self)
        
        self.setObjectName("leftJointControl")
        
        self._joint_id = -1
        self._joint_angle_inc = 0
        self._speed = 1.0
        
        self._joint_states = [0.0,0.0,0.0,0.0,0.0,0.0]
        self._joint_offset = [0.0,0.0,0.0,0.0,0.0,0.0]
        self._joint_states_comm = QAgiSubscriber(self,
                                                '/joint_states',
                                                JointState,
                                                self._update_current_joint_states,
                                                max_rate = 30)
        
        
        
        self.joint_a1 = RobotAxisWidget(self.AXIS_MIN_LIMITS[0],self.AXIS_MAX_LIMITS[0], 176)
        self.joint_a1.setMaximumHeight(60)
        
        self.joint_a2 = RobotAxisWidget(self.AXIS_MIN_LIMITS[1],self.AXIS_MAX_LIMITS[1], 176)
        self.joint_a2.setMaximumHeight(60)
        
        self.joint_a3 = RobotAxisWidget(self.AXIS_MIN_LIMITS[2],self.AXIS_MAX_LIMITS[2], 176)
        self.joint_a3.setMaximumHeight(60)
        
        self.joint_a4 = RobotAxisWidget(self.AXIS_MIN_LIMITS[3],self.AXIS_MAX_LIMITS[3], 176)
        self.joint_a4.setMaximumHeight(60)
        
        self.joint_a5 = RobotAxisWidget(self.AXIS_MIN_LIMITS[4],self.AXIS_MAX_LIMITS[4], 176)
        self.joint_a5.setMaximumHeight(60)
        
        self.joint_a6 = RobotAxisWidget(self.AXIS_MIN_LIMITS[5],self.AXIS_MAX_LIMITS[5], 176)
        self.joint_a6.setMaximumHeight(60)
        
        self.joint_a1_layout.addWidget(self.joint_a1)
        self.joint_a2_layout.addWidget(self.joint_a2)
        self.joint_a3_layout.addWidget(self.joint_a3)
        self.joint_a4_layout.addWidget(self.joint_a4)
        self.joint_a5_layout.addWidget(self.joint_a5)
        self.joint_a6_layout.addWidget(self.joint_a6)
        
        for i in range(1,7):
            
            jrb = getattr(self, "joint_a%i_right_button"%i)
            jlb = getattr(self, "joint_a%i_left_button"%i)
            
            jrb.setIcon(R.getIconById("ico_plus"))
            jrb.setIconSize(QSize(40,40))
            
            jlb.setIcon(R.getIconById("ico_minus"))
            jlb.setIconSize(QSize(40,40))
            
            self.connect(jrb,SIGNAL("pressed()"), self._joint_move)
            self.connect(jrb,SIGNAL("released()"), self._slot_reset_offset)
            
            self.connect(jlb,SIGNAL("pressed()"), self._joint_move)
            self.connect(jlb,SIGNAL("released()"), self._slot_reset_offset)
            
            
        self._reset_offset = QTimer(self)
        self.connect(self._reset_offset, SIGNAL("timeout()"), self._slot_reset_offset)
        self._pressed_timer = QTimer(self)
        self.connect(self._pressed_timer, SIGNAL("timeout()"), self._slot_publish_move)
        
        self._command_publisher = rospy.Publisher("/aldo/articular_cmd", ArticularGoals, queue_size=1)
        

        
    def subscribe(self):
        self._speed = 1.0
        self._joint_states_comm.subscribe() 
        self._reset_offset.start(150)  
        
    def unregister(self):
        self._joint_states_comm.unregister()
        self._reset_offset.stop()
        self._pressed_timer.stop()
    
    def _get_joint_id_clicked(self, sender_id):
        
        for i in range(1,7):
            
            jrb = getattr(self, "joint_a%i_right_button"%i)
            jlb = getattr(self, "joint_a%i_left_button"%i)
            
            if jrb.objectName() == sender_id.objectName():
                return (i-1, 1)
            elif jlb.objectName() == sender_id.objectName():
                return (i-1, -1)
            else:
                continue
            
        return (-1, 0)
    
    def _joint_move(self):
        self._joint_id, self._joint_angle_inc = self._get_joint_id_clicked(self.sender())
        self._reset_offset.stop()
        self._pressed_timer.start(1000.0/self.PUBLISH_FREQUENCY)
        
    def _slot_publish_move(self):
        self._joint_offset[self._joint_id] = self._joint_offset[self._joint_id] + self._joint_angle_inc*self._speed
        
        if(self._joint_offset[self._joint_id] > self.AXIS_MAX_LIMITS[self._joint_id]):
            self._joint_offset[self._joint_id] = self.AXIS_MAX_LIMITS[self._joint_id]
        if(self._joint_offset[self._joint_id] < self.AXIS_MIN_LIMITS[self._joint_id]):
            self._joint_offset[self._joint_id] = self.AXIS_MIN_LIMITS[self._joint_id]
        
        if(self._speed < self.SPEED_THRESHOLD):
            self._speed = self._speed + self.SPEED_INCREASED*(1/self.PUBLISH_FREQUENCY)

        position_target = [math.radians(self._joint_offset[0]),
                           math.radians(self._joint_offset[1]),
                           math.radians(self._joint_offset[2]),
                           math.radians(self._joint_offset[3]),
                           math.radians(self._joint_offset[4]),
                           math.radians(self._joint_offset[5])]
        
        msg = ArticularGoals()
        msg.header.stamp = rospy.Time.now()
        msg.right_arm.use.data = False
        msg.right_arm.name = ["","","","","",""]
        msg.right_arm.position = [0.0,0.0,0.0,0.0,0.0,0.0]
        msg.left_arm.use.data = True
        msg.left_arm.name = self.AXIS_NAME
        msg.left_arm.position = position_target
        
        
        self._command_publisher.publish(msg)
        
    def _slot_reset_offset(self):
        self._speed = 1.0
        self._pressed_timer.stop()
        self._reset_offset.start(150)
        self._joint_offset = self._joint_states
        
    def _update_current_joint_states(self, states):
        ##find joints value
        for i_jt in range(len(self.AXIS_NAME)):
            for j_jt in range(len(states.position)):
                if(self.AXIS_NAME[i_jt] == states.name[j_jt]):
                    jangle = getattr(self, "joint_a%i"%(i_jt+1))
                    jangle.setJointAngle(math.degrees(states.position[j_jt]))
                    self._joint_states[i_jt] = math.degrees(states.position[j_jt])
    
                    
class RightJointControlUi(QWidget):
    
    AXIS_NAME = ["right_shoulder_pan_joint","right_shoulder_lift_joint","right_elbow_joint", 
                  "right_wrist_1_joint","right_wrist_2_joint", "right_wrist_3_joint"]
    
    AXIS_MIN_LIMITS = [math.degrees(-1*math.pi/4),
                       math.degrees(-1*(1+math.pi)),
                       math.degrees(-1*math.pi),
                       math.degrees(-1*math.pi),
                       math.degrees(-1*math.pi),
                       math.degrees(-1*math.pi)]
    AXIS_MAX_LIMITS = [math.degrees(math.pi),
                       math.degrees(0.0),
                       math.degrees(math.pi),
                       math.degrees(math.pi),
                       math.degrees(math.pi),
                       math.degrees(math.pi)]
    
    SPEED_THRESHOLD = 5.0
    PUBLISH_FREQUENCY = 10.0
    SPEED_INCREASED = 1.0
    
    def __init__(self):
        QWidget.__init__(self)
        
        loadUi(R.layouts.joint_control, self)
        
        self.setObjectName("rightJointControl")
        
        self._joint_id = -1
        self._joint_angle_inc = 0
        self._speed = 1.0
        
        self._joint_states = [0.0,0.0,0.0,0.0,0.0,0.0]
        self._joint_offset = [0.0,0.0,0.0,0.0,0.0,0.0]
        self._joint_states_comm = QAgiSubscriber(self,
                                                '/joint_states',
                                                JointState,
                                                self._update_current_joint_states,
                                                max_rate = 30)
        
        
        
        self.joint_a1 = RobotAxisWidget(self.AXIS_MIN_LIMITS[0],self.AXIS_MAX_LIMITS[0], 176)
        self.joint_a1.setMaximumHeight(60)
        
        self.joint_a2 = RobotAxisWidget(self.AXIS_MIN_LIMITS[1],self.AXIS_MAX_LIMITS[1], 176)
        self.joint_a2.setMaximumHeight(60)
        
        self.joint_a3 = RobotAxisWidget(self.AXIS_MIN_LIMITS[2],self.AXIS_MAX_LIMITS[2], 176)
        self.joint_a3.setMaximumHeight(60)
        
        self.joint_a4 = RobotAxisWidget(self.AXIS_MIN_LIMITS[3],self.AXIS_MAX_LIMITS[3], 176)
        self.joint_a4.setMaximumHeight(60)
        
        self.joint_a5 = RobotAxisWidget(self.AXIS_MIN_LIMITS[4],self.AXIS_MAX_LIMITS[4], 176)
        self.joint_a5.setMaximumHeight(60)
        
        self.joint_a6 = RobotAxisWidget(self.AXIS_MIN_LIMITS[5],self.AXIS_MAX_LIMITS[5], 176)
        self.joint_a6.setMaximumHeight(60)
        
        self.joint_a1_layout.addWidget(self.joint_a1)
        self.joint_a2_layout.addWidget(self.joint_a2)
        self.joint_a3_layout.addWidget(self.joint_a3)
        self.joint_a4_layout.addWidget(self.joint_a4)
        self.joint_a5_layout.addWidget(self.joint_a5)
        self.joint_a6_layout.addWidget(self.joint_a6)
        
        for i in range(1,7):
            
            jrb = getattr(self, "joint_a%i_right_button"%i)
            jlb = getattr(self, "joint_a%i_left_button"%i)
            
            jrb.setIcon(R.getIconById("ico_plus"))
            jrb.setIconSize(QSize(40,40))
            
            jlb.setIcon(R.getIconById("ico_minus"))
            jlb.setIconSize(QSize(40,40))
            
            self.connect(jrb,SIGNAL("pressed()"), self._joint_move)
            self.connect(jrb,SIGNAL("released()"), self._slot_reset_offset)
            
            self.connect(jlb,SIGNAL("pressed()"), self._joint_move)
            self.connect(jlb,SIGNAL("released()"), self._slot_reset_offset)
            
            
        self._reset_offset = QTimer(self)
        self.connect(self._reset_offset, SIGNAL("timeout()"), self._slot_reset_offset)
        self._pressed_timer = QTimer(self)
        self.connect(self._pressed_timer, SIGNAL("timeout()"), self._slot_publish_move)
        
        self._command_publisher = rospy.Publisher("/aldo/articular_cmd", ArticularGoals, queue_size=1)
        

        
    def subscribe(self):
        self._speed = 1.0
        self._joint_states_comm.subscribe() 
        self._reset_offset.start(150)  
        
    def unregister(self):
        self._joint_states_comm.unregister()
        self._reset_offset.stop()
        self._pressed_timer.stop()
    
    def _get_joint_id_clicked(self, sender_id):
        
        for i in range(1,7):
            
            jrb = getattr(self, "joint_a%i_right_button"%i)
            jlb = getattr(self, "joint_a%i_left_button"%i)
            
            if jrb.objectName() == sender_id.objectName():
                return (i-1, 1)
            elif jlb.objectName() == sender_id.objectName():
                return (i-1, -1)
            else:
                continue
            
        return (-1, 0)
    
    def _joint_move(self):
        self._joint_id, self._joint_angle_inc = self._get_joint_id_clicked(self.sender())
        self._reset_offset.stop()
        self._pressed_timer.start(1000.0/self.PUBLISH_FREQUENCY)
        
    def _slot_publish_move(self):
        self._joint_offset[self._joint_id] = self._joint_offset[self._joint_id] + self._joint_angle_inc*self._speed
        
        if(self._joint_offset[self._joint_id] > self.AXIS_MAX_LIMITS[self._joint_id]):
            self._joint_offset[self._joint_id] = self.AXIS_MAX_LIMITS[self._joint_id]
        if(self._joint_offset[self._joint_id] < self.AXIS_MIN_LIMITS[self._joint_id]):
            self._joint_offset[self._joint_id] = self.AXIS_MIN_LIMITS[self._joint_id]
            
        if(self._speed < self.SPEED_THRESHOLD):
            self._speed = self._speed + self.SPEED_INCREASED*(1/self.PUBLISH_FREQUENCY)
                 
        
        msg = ArticularGoals()
        msg.header.stamp = rospy.Time.now()
        msg.right_arm.use.data = True
        msg.right_arm.name = self.AXIS_NAME
        position_target = [math.radians(self._joint_offset[0]),
                           math.radians(self._joint_offset[1]),
                           math.radians(self._joint_offset[2]),
                           math.radians(self._joint_offset[3]),
                           math.radians(self._joint_offset[4]),
                           math.radians(self._joint_offset[5])]
        msg.right_arm.position = position_target
        msg.left_arm.use.data = False
        msg.left_arm.name = ["","","","","",""]
        msg.left_arm.position = [0.0,0.0,0.0,0.0,0.0,0.0]
        self._command_publisher.publish(msg)
        
    def _slot_reset_offset(self):
        self._speed = 1.0
        self._pressed_timer.stop()
        self._reset_offset.start(150)
        self._joint_offset = self._joint_states
        
    def _update_current_joint_states(self, states):
        ##find joints value
        for i_jt in range(len(self.AXIS_NAME)):
            for j_jt in range(len(states.position)):
                if(self.AXIS_NAME[i_jt] == states.name[j_jt]):
                    jangle = getattr(self, "joint_a%i"%(i_jt+1))
                    jangle.setJointAngle(math.degrees(states.position[j_jt]))
                    self._joint_states[i_jt] = math.degrees(states.position[j_jt])
                
            
            
        
if __name__ == "__main__":
    
    from python_qt_binding.QtGui import QApplication
    import sys
    import rospy
    
    rospy.init_node('aldo_plugin_test_2')
    
    app = QApplication(sys.argv)
    
    MainWindow = RightJointControlUi()
    MainWindow.subscribe()
    MainWindow.show()
    
    app.exec_()
    
    MainWindow.unregister()
