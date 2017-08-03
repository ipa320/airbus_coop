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
import tf

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from cobot_gui import plugin, ControlMode, EmergencyStopState
from rviz_robot import RVizRobot
from ur10_plugin.res import R
from ur10_driver.msg  import UR10Cartesian
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from ur10_driver.msg  import UR10Articular
from std_msgs.msg import Int16
from std_msgs.msg import Empty
from xml.etree.ElementTree import ElementTree
from xml.etree.ElementTree import Element
from  lxml import etree
from PyQt4 import QtGui, QtCore

class Ur10Plugin(plugin.Plugin):
    
    def __init__(self, context):

        plugin.Plugin.__init__(self, context)
        
    def onCreate(self, param):

        # Load ui file
        loadUi(R.layouts.mainwindow, self)
        
        
        self.target_point_pub = None 
        self.pose_cart_sub = None
        self.joint_sub = None
        self.target_pt =  UR10Cartesian()
        self.root = etree.Element('root')
        self.tree=etree.ElementTree(self.root)
        self.compteur_point=1
        self.initialisation_xml()
        self._robot_ui = RVizRobot(self)
        robot_widget = self._robot_ui.get_widget()
        self.robot_view.setWidget(robot_widget)

    def ur10_pose_cart_cb(self, msg):
        self.label.setText("      x : %.3f" %(msg.pose.position.x))
        self.label_2.setText("      y : %.3f" %(msg.pose.position.y))
        self.label_3.setText("      z : %.3f" %(msg.pose.position.z))
        self.label_6.setText("      x : %.3f" %(msg.pose.orientation.x))
        self.label_7.setText("      y : %.3f" %(msg.pose.orientation.y))
        self.label_8.setText("      z : %.3f" %(msg.pose.orientation.z))
        self.label_9.setText("      w : %.3f" %(msg.pose.orientation.w))
        
        self.target_pt.header=msg.header
        self.target_pt.pose.position.x = msg.pose.position.x
        self.target_pt.pose.position.y = msg.pose.position.y
        self.target_pt.pose.position.z = msg.pose.position.z
        self.target_pt.pose.orientation.x = msg.pose.orientation.x
        self.target_pt.pose.orientation.y = msg.pose.orientation.y
        self.target_pt.pose.orientation.z = msg.pose.orientation.z
        self.target_pt.pose.orientation.w = msg.pose.orientation.w
        self.target_pt.tool         = int(msg.header.frame_id)
        self.target_pt.speed        = 0.1
        self.target_pt.acceleration = 0.1
        self.angle=tf.transformations.euler_from_quaternion([self.target_pt.pose.orientation.x,self.target_pt.pose.orientation.y,self.target_pt.pose.orientation.z,self.target_pt.pose.orientation.w], axes='sxyz')
  
    def ur10_joint_state_cb(self, msg):
        self.label_11.setText("      Joint 1 : %.3f" %(msg.position[0]))
        self.label_12.setText("      Joint 2 : %.3f" %(msg.position[1]))
        self.label_13.setText("      Joint 3 : %.3f" %(msg.position[2]))
        self.label_14.setText("      Joint 4 : %.3f" %(msg.position[3]))
        self.label_15.setText("      Joint 5 : %.3f" %(msg.position[4]))
        self.label_16.setText("      Joint 6 : %.3f" %(msg.position[5]))
           
    def onResume(self):
        pass
    
    def onPause(self):
        if self.pose_cart_sub is not None:
            self.pose_cart_sub.unregister()
            self.pose_cart_sub = None
            
        if self.joint_sub is not None:
            self.joint_sub.unregister()
            self.joint_sub = None
            
        if self.target_point_pub is not None:
            self.target_point_pub.unregister()
            self.target_point_pub = None
        pass
    
    def action_increment_position_x(self):
       self.target_pt.pose.position.x = self.target_pt.pose.position.x+0.01
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_decrement_position_x(self):
       self.target_pt.pose.position.x = self.target_pt.pose.position.x-0.01
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_increment_position_y(self):
       self.target_pt.pose.position.y = self.target_pt.pose.position.y+0.01
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_decrement_position_y(self):
       self.target_pt.pose.position.y = self.target_pt.pose.position.y-0.01
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_increment_position_z(self):
       self.target_pt.pose.position.z = self.target_pt.pose.position.z+0.01
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_decrement_position_z(self):
       self.target_pt.pose.position.z = self.target_pt.pose.position.z-0.01
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_increment_orientation_x(self):
       new_angle = self.angle[2]-0.01
       self.quaternion=tf.transformations.quaternion_from_euler(self.angle[0], self.angle[1], new_angle, axes='sxyz')
       self.target_pt.pose.orientation.x=self.quaternion[0]
       self.target_pt.pose.orientation.y=self.quaternion[1]
       self.target_pt.pose.orientation.z=self.quaternion[2]
       self.target_pt.pose.orientation.w=self.quaternion[3]
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_decrement_orientation_x(self):
       new_angle = self.angle[2]+0.01
       self.quaternion=tf.transformations.quaternion_from_euler(self.angle[0], self.angle[1], new_angle, axes='sxyz')
       self.target_pt.pose.orientation.x=self.quaternion[0]
       self.target_pt.pose.orientation.y=self.quaternion[1]
       self.target_pt.pose.orientation.z=self.quaternion[2]
       self.target_pt.pose.orientation.w=self.quaternion[3]
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_increment_orientation_y(self):
       new_angle = self.angle[1]-0.01
       self.quaternion=tf.transformations.quaternion_from_euler(self.angle[0], new_angle, self.angle[2], axes='sxyz')
       self.target_pt.pose.orientation.x=self.quaternion[0]
       self.target_pt.pose.orientation.y=self.quaternion[1]
       self.target_pt.pose.orientation.z=self.quaternion[2]
       self.target_pt.pose.orientation.w=self.quaternion[3]
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_decrement_orientation_y(self):
       new_angle = self.angle[1]+0.01
       self.quaternion=tf.transformations.quaternion_from_euler(self.angle[0], new_angle, self.angle[2], axes='sxyz')
       self.target_pt.pose.orientation.x=self.quaternion[0]
       self.target_pt.pose.orientation.y=self.quaternion[1]
       self.target_pt.pose.orientation.z=self.quaternion[2]
       self.target_pt.pose.orientation.w=self.quaternion[3]
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_increment_orientation_z(self):
       new_angle = self.angle[0]-0.01
       self.quaternion=tf.transformations.quaternion_from_euler(new_angle, self.angle[1], self.angle[2], axes='sxyz')
       self.target_pt.pose.orientation.x=self.quaternion[0]
       self.target_pt.pose.orientation.y=self.quaternion[1]
       self.target_pt.pose.orientation.z=self.quaternion[2]
       self.target_pt.pose.orientation.w=self.quaternion[3]
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
       
    def action_decrement_orientation_z(self):
       new_angle = self.angle[0]+0.01
       self.quaternion=tf.transformations.quaternion_from_euler(new_angle, self.angle[1], self.angle[2], axes='sxyz')
       self.target_pt.pose.orientation.x=self.quaternion[0]
       self.target_pt.pose.orientation.y=self.quaternion[1]
       self.target_pt.pose.orientation.z=self.quaternion[2]
       self.target_pt.pose.orientation.w=self.quaternion[3]
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)      
       
    def Home(self):
       self.target_pt.pose.position.x=-0.40812
       self.target_pt.pose.position.y=0.18693
       self.target_pt.pose.position.z=0.00383
       self.target_pt.pose.orientation.x=-0.707997739217
       self.target_pt.pose.orientation.y=0.70615895848
       self.target_pt.pose.orientation.z=0.00504075356544
       self.target_pt.pose.orientation.w=0.00730187819768
       if(self.target_pt == None or (self.target_pt.pose.position.x==0 and self.target_pt.pose.position.y==0 and self.target_pt.pose.position.z==0 and self.target_pt.pose.orientation.x==0 and self.target_pt.pose.orientation.y==0 and self.target_pt.pose.orientation.z==0 and self.target_pt.pose.orientation.w==0)):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
       self.target_point_pub.publish(self.target_pt)
    
    def initialisation_xml(self):
        
        # another child with text
        Home = etree.Element('Home')
        Home.set('type','cart')
        self.root.append(Home)
        position = etree.Element('position')
        position.set('x','-0.40812')
        position.set('y','0.18693')
        position.set('z','0.00383')
        Home.append(position)
        
        orientation = etree.Element('orientation')
        orientation.set('x','-0.707997739217')
        orientation.set('y','0.70615895848')
        orientation.set('z','0.00504075356544')
        orientation.set('w','0.00730187819768')
        orientation.set('type','quat')
        Home.append(orientation)
        
        self.tree.write(open(r'/opt/ros/indigo/workspace/src/applications/trajectory_test.xml','w'))
       
    def acquisition(self):
        
        Pt1 = etree.Element('Pt%s' %self.compteur_point)
        Pt1.set('type','cart')
        self.root.append(Pt1)
        position = etree.Element('position')
        position.set('x','%s' %self.target_pt.pose.position.x) 
        position.set('y','%s' %self.target_pt.pose.position.y)
        position.set('z','%s' %self.target_pt.pose.position.z)
        Pt1.append(position)
        
        orientation = etree.Element('orientation')
        orientation.set('x','%s' %self.target_pt.pose.orientation.x)
        orientation.set('y','%s' %self.target_pt.pose.orientation.y)
        orientation.set('z','%s' %self.target_pt.pose.orientation.z)
        orientation.set('w','%s' %self.target_pt.pose.orientation.w)
        orientation.set('type','quat')
        Pt1.append(orientation)

        self.tree.write(open(r'/opt/ros/indigo/workspace/src/applications/trajectory_test.xml','w'))
        self.compteur_point+=1
    
    def onResume(self):

        self.pose_cart_sub = rospy.Subscriber("/ur10/pose_cart",PoseStamped,self.ur10_pose_cart_cb)
        self.joint_sub = rospy.Subscriber("/ur10/joint_state", JointState,self.ur10_joint_state_cb)
        self.target_point_pub   = rospy.Publisher("/ur10/target_point",UR10Cartesian,queue_size=1)
        
        self.pushButton_2.clicked.connect(self.action_increment_position_x)
        self.pushButton.clicked.connect(self.action_decrement_position_x)
        
        self.pushButton_4.clicked.connect(self.action_increment_position_y)
        self.pushButton_3.clicked.connect(self.action_decrement_position_y)
        
        self.pushButton_5.clicked.connect(self.action_increment_position_z)
        self.pushButton_6.clicked.connect(self.action_decrement_position_z)
        
        self.pushButton_10.clicked.connect(self.action_increment_orientation_x)
        self.pushButton_9.clicked.connect(self.action_decrement_orientation_x)
        
        self.pushButton_12.clicked.connect(self.action_increment_orientation_y)
        self.pushButton_11.clicked.connect(self.action_decrement_orientation_y)
        
        self.pushButton_8.clicked.connect(self.action_increment_orientation_z)
        self.pushButton_7.clicked.connect(self.action_decrement_orientation_z)
        
        self.pushButton_13.clicked.connect(self.acquisition)
        
        self.pushButton_14.clicked.connect(self.Home)
        
    
    def onControlModeChanged(self, mode):
        """ This method is called when the user change the control mode.
        @param mode: C{ControlMode}
        
        You can write rules when the control mode changes.
        
        Example:
        NB: import needed (from cobot_gui import ControlMode)
        
        if mode == ControlMode.AUTOMATIC:
            # Do somethings
        elif mode == ControlMode.MANUAL:
            # Do somethings
        """
        
    def onUserChanged(self, user):
        """ This method is called when a new user logged-in.
        @param user_info: C{User}
        
        You can define different behaviors according to the type of user.
        
        Example:
        NB: import needed (from cobot_gui import UserPrivilege)
        
        if user.getUserPrivilege() == UserPrivilege.DEVELOPER:
            # Do some rules
        else:
            # Do some rules
        """
    
    def onTranslate(self, lng):
        """ This method is called when the user change the language.
        You can retranslate your plugin here.
        """
        # Exemple:
       
    
    def onEmergencyStop(self, state):
        """ This method is called when the emergency stop state changed.
        You can make emergency action here.
        """
        # Default emergency routine :
        if state == EmergencyStopState.LOCKED:
            self.onPause()
        elif state == EmergencyStopState.UNLOCKED:
            self.onResume()
    
    def onDestroy(self):
        """ This method is called when cobot_gui closes.
        You can free memory and disconnects topics
        """
        # Default exit routine :
        self.onPause()
        
if __name__ == "__main__":
    
    import sys
    
    rospy.init_node("ur10_plugin_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("ur10_plugin", Ur10Plugin, "en")
    window.setWindowTitle("Ur10Plugin")
    window.show()
    a.exec_()
