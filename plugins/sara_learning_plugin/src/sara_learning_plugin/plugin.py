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

from std_msgs.msg import Empty, Int8

from cobot_gui import plugin, ControlMode, EmergencyStopState

#from common_lib.agi_sound import PlaySound

from sara_learning_plugin.res import R

class SaraLearningPlugin(plugin.Plugin):
    
    ACTION = {"gravity_on":0,
              "gravity_off":1,
              "learn_platform":2,
              "learn_articular":3,
              "learn_cartesian":4,
              "stop":5
        }
    
    def __init__(self, context):
        """ Plugin constructor.
        It would be better not to change anything here.
        """
        plugin.Plugin.__init__(self, context)
        
    def onCreate(self, param):
        # Load ui file
        loadUi(R.layouts.mainwindow, self)
        self.learning_on = False
        
        self.connect(self.gravity_on, SIGNAL("pressed()"), self._slot_gravity_on)
        self.connect(self.gravity_off, SIGNAL("pressed()"), self._slot_gravity_off)
        self.connect(self.learn_platform, SIGNAL("pressed()"), self._slot_learn_platform)
        self.connect(self.learn_articular, SIGNAL("pressed()"), self._slot_learn_articular)
        self.connect(self.learn_cartesian, SIGNAL("pressed()"), self._slot_learn_cartesian)
        self.connect(self.stop_learn, SIGNAL("pressed()"), self._slot_stop_learning)
        
        
        self.learning_start_sub = rospy.Subscriber("/learning/start", Empty, self._learning_start_cb)
        self.learning_arm_sub = rospy.Subscriber("/learning/arm_on", Empty, self._learning_arm_cb)
        self.learning_nav_sub = rospy.Subscriber("/learning/nav_on", Empty, self._learning_nav_cb)
        
        self._action_publisher = rospy.Publisher("/learning/action", Int8, queue_size=1)
        self.reset()
    '''def all _slot'''
    def _slot_gravity_on(self):
        self.gravity_status.setText("ON")
        self.gravity_status.setStyleSheet("color: green")
        self._action_publisher.publish(0)
        rospy.sleep(0.1)
        
    def _slot_gravity_off(self):
        self.gravity_status.setText("OFF")
        self.gravity_status.setStyleSheet("color: red")
        self._action_publisher.publish(1)
        rospy.sleep(0.1)
        
    def _slot_learn_platform(self):
        self._action_publisher.publish(2)
        #PlaySound(R.sound.beep08b)
        rospy.sleep(0.1)
        
    def _slot_learn_articular(self):
        self._action_publisher.publish(3)
        #PlaySound(R.sound.beep08b)
        rospy.sleep(0.1)
        
    def _slot_learn_cartesian(self):
        self._action_publisher.publish(4)
        #PlaySound(R.sound.beep08b)
        rospy.sleep(0.1)
        
    def _slot_stop_learning(self):
        self._action_publisher.publish(5)
        rospy.sleep(0.1)
        self.reset()
        
    ''' def all cb'''
        
    def _learning_start_cb(self, msg):
        if(self.learning_on == False):
            self.learning_on = True
            self.learning_status.setText("ON")
            self.learning_status.setStyleSheet("color: green")
            self.stop_learn.setEnabled(True)
        
    def _learning_arm_cb(self, msg):
        if(self.learning_on == True):
            self.lwr_status.setText("ON")
            self.lwr_status.setStyleSheet("color: green")
            self.gravity_on.setEnabled(True)
            self.gravity_off.setEnabled(True)
            self.learn_articular.setEnabled(True)
            self.learn_cartesian.setEnabled(True)
        
    def _learning_nav_cb(self, msg):
        if(self.learning_on == True):
            self.platform_status.setText("ON")
            self.platform_status.setStyleSheet("color: green")
            self.learn_platform.setEnabled(True)
    
    def reset(self):
        self.gravity_status.setText("OFF")
        self.gravity_status.setStyleSheet("color: red")
        self.platform_status.setText("OFF")
        self.platform_status.setStyleSheet("color: red")
        self.learning_status.setText("OFF")
        self.learning_status.setStyleSheet("color: red")
        self.lwr_status.setText("OFF")
        self.lwr_status.setStyleSheet("color: red")
        self.learning_on = False
        self.learn_platform.setEnabled(False)
        self.stop_learn.setEnabled(False)
        self.gravity_on.setEnabled(False)
        self.gravity_off.setEnabled(False)
        self.learn_articular.setEnabled(False)
        self.learn_cartesian.setEnabled(False)
    
    def onPause(self):
        pass
    
    def onResume(self):
        pass
    
    def onControlModeChanged(self, mode):
       pass
        
    def onUserChanged(self, user):
       pass
    
    def onTranslate(self, lng):
        pass
    
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
    
    rospy.init_node("sara_learning_plugin_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("sara_learning_plugin", SaraLearningPlugin, "en")
    window.setWindowTitle("SaraLearningPlugin")
    window.show()
    a.exec_()
