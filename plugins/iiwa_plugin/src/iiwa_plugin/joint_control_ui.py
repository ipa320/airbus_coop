#!/usr/bin/env python

import rospy
import os
import math
import threading

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_axis_widget import RobotAxisWidget

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from iiwa_plugin.res import R

class JointControlUi(QWidget):
    
    LIMITS_AXIS = [[-169.,169.], [-119.,119.], [-169.,169.],
                   [-119.,119.], [-169.,169.], [-119.,119.],
                   [-174.,174.]]
    
    def __init__(self, servo_motion):
        QWidget.__init__(self)
        
        loadUi(R.layouts.joint_control, self)
        
        self.setObjectName("jointControl")
        
        self._servo_motion = servo_motion
        
        self._joint_id = -1
        self._joint_angle_inc = 0
        self._joint_angle_vel = 15.
        self._curr_sender_id = ""
        
        self._joint_states = JointState()
        self._joint_states_comm = QAgiSubscriber(self,
                                                '/iiwa/joint_states',
                                                JointState,
                                                self._update_current_joint_states,
                                                max_rate = 30)
        
        self.joint_a1 = RobotAxisWidget(170, 176)
        self.joint_a1.setMaximumHeight(60)
        
        self.joint_a2 = RobotAxisWidget(120, 176)
        self.joint_a2.setMaximumHeight(60)
        
        self.joint_a3 = RobotAxisWidget(170, 110)
        self.joint_a3.setMaximumHeight(60)
        
        self.joint_a4 = RobotAxisWidget(120, 110)
        self.joint_a4.setMaximumHeight(60)
        
        self.joint_a5 = RobotAxisWidget(170, 110)
        self.joint_a5.setMaximumHeight(60)
        
        self.joint_a6 = RobotAxisWidget(120, 40)
        self.joint_a6.setMaximumHeight(60)
        
        self.joint_a7 = RobotAxisWidget(175, 40)
        self.joint_a7.setMaximumHeight(60)
        
        self.joint_a1_layout.addWidget(self.joint_a1)
        self.joint_a2_layout.addWidget(self.joint_a2)
        self.joint_a3_layout.addWidget(self.joint_a3)
        self.joint_a4_layout.addWidget(self.joint_a4)
        self.joint_a5_layout.addWidget(self.joint_a5)
        self.joint_a6_layout.addWidget(self.joint_a6)
        self.joint_a7_layout.addWidget(self.joint_a7)
        
        for i in range(1,8):
            
            jrb = getattr(self, "joint_a%i_right_button"%i)
            jlb = getattr(self, "joint_a%i_left_button"%i)
            
            jrb.setIcon(R.getIconById("ico_plus"))
            jrb.setIconSize(QSize(40,40))
            
            jlb.setIcon(R.getIconById("ico_minus"))
            jlb.setIconSize(QSize(40,40))
            
            self.connect(jrb,SIGNAL("pressed()"), self._joint_move)
            self.connect(jrb,SIGNAL("released()"), self.stop_motion)
            
            self.connect(jlb,SIGNAL("pressed()"), self._joint_move)
            self.connect(jlb,SIGNAL("released()"), self.stop_motion)
            
            
        self._trigger_pub = QTimer(self)
        self.connect(self._trigger_pub, SIGNAL("timeout()"), self._publish_joint_command)
        
    def subscribe(self):
        self._joint_states_comm.subscribe()
        
        
    def unregister(self):
        self._joint_states_comm.unregister()
    
    def _get_joint_id_clicked(self, sender_id):
        
        self._curr_sender_id = sender_id.objectName()
        
        for i in range(1,8):
            
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
        self._trigger_pub.start(200)
        
    def stop_motion(self):
        self._trigger_pub.stop()
        self._servo_motion.stopMotion();
        
    def _publish_joint_command(self):
        
        if self._joint_id < 0:
            return
        
        joint = JointTrajectoryPoint()
        joint.positions = 7*[0.]
        
        if len(self._joint_states.position) >= 7:
            for i in range(0, 7):
                joint.positions[i] = self._joint_states.position[i]
                
        step = math.radians(self._joint_angle_inc*self._joint_angle_vel)
        jcurr = joint.positions[self._joint_id]
        jdest = joint.positions[self._joint_id]
        jdest += step
        
        direction = 0
        
        if (jdest - jcurr) > 0:
            direction = 1
        else:
            direction = -1
            
        motion = False
        # Joint agnle in [MIN, MAX]
        jdest_deg = math.degrees(jdest)
        
        if jdest_deg > self.LIMITS_AXIS[self._joint_id][0] and \
           jdest_deg < self.LIMITS_AXIS[self._joint_id][1]:
             motion = True
        else:
            if jdest_deg < self.LIMITS_AXIS[self._joint_id][0] and direction == 1:
                motion = True
            else:
                motion = False
            if jdest_deg > self.LIMITS_AXIS[self._joint_id][1] and direction == -1:
                motion = True
            else:
                motion = False
            
        if motion:
            print joint
            joint.positions[self._joint_id] = jdest 
            self._servo_motion.moveAsync(joint)
        
    def _update_current_joint_states(self, states):
        
        self._joint_states = states
        
        for i in range(0,7):
            jangle = getattr(self, "joint_a%i"%(i+1))
            jangle.setJointAngle(math.degrees(self._joint_states.position[i]))
            jangle.setJointTorque(self._joint_states.effort[i])
        
if __name__ == "__main__":
    
    from python_qt_binding.QtGui import QApplication
    import sys
    import rospy
    
    rospy.init_node('iiwa_plugin_test_2')
    
    app = QApplication(sys.argv)
    
    MainWindow = JointControlUi()
    MainWindow.subscribe()
    MainWindow.show()
    
    app.exec_()
    
    MainWindow.unregister()
