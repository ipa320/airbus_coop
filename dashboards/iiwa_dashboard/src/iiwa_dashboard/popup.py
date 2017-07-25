#!/usr/bin/env python

## @package: robot_dashboard
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/04/2015

import os
import rospy
from industrial_msgs.msg import RobotStatus, TriState
from arm_msgs.msg import IOStates

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from cobot_gui import dashboard

from iiwa_dashboard.res import R

class IIWADashboardPopup(dashboard.DashboardPopup):
    
    def __init__(self, parent):
        dashboard.DashboardPopup.__init__(self, parent)
        
    def onCreate(self, param):
        
        loadUi(R.layouts.popup, self)
        
        self.setRelativePosition(dashboard.DashboardPopup.TopRight,
                                 dashboard.DashboardPopup.BottomRight)
        
        self._robot_status_sub = QAgiSubscriber(self,
                                              '/iiwa/status',
                                              RobotStatus,
                                              self._update_robot_status_,
                                              self._robot_status_timeout_,
                                              timeout  = rospy.Duration(1),
                                              max_rate = 3)
        
        self._robot_io_sub = QAgiSubscriber(self,
                                            '/iiwa/io_states',
                                            IOStates,
                                            self._update_robot_io_,
                                            max_rate = 3)
        
        self._status_led = None
        self._io_led = None
        
        self.px_comm_green = R.getPixmapById("comm_green").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_comm_grey  = R.getPixmapById("comm_grey").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_comm_red   = R.getPixmapById("comm_red").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        
        self.px_power_green = R.getPixmapById("power_green").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_power_grey  = R.getPixmapById("power_grey").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_power_red   = R.getPixmapById("power_red").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        
        self.px_error_green = R.getPixmapById("error_green").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_error_grey  = R.getPixmapById("error_grey").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_error_red   = R.getPixmapById("error_red").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        
        self.px_base    = R.getPixmapById("base").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_tool    = R.getPixmapById("tool").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_led_off = R.getPixmapById("led_off").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.px_led_on  = R.getPixmapById("led_on").scaled(32, 32, Qt.KeepAspectRatio,Qt.SmoothTransformation)
        
        self.communication_state.setPixmap(self.px_comm_grey)
        
        self.power_state.setPixmap(self.px_power_grey)
        
        self.error_state.setPixmap(self.px_error_grey)
        
        self.base_frame_ico.setPixmap(self.px_base)
        
        self.tool_frame_ico.setPixmap(self.px_tool)
        
        self._io_led = {0 : self.px_led_off,
                        1 : self.px_led_on}
        
        for i in range(1,9):
            input = getattr(self, 'digital_input_%i'%i)
            input.setPixmap(self.px_led_off)
            
        for i in range(1,5):
            output = getattr(self, 'digital_output_%i'%i)
            output.setPixmap(self.px_led_off)
    
    def _update_robot_status_(self, status):
        
        self.communication_state.setPixmap(self.px_comm_green)
        
        if status.drives_powered.val == 1:
            self.power_state.setPixmap(self.px_power_green)
        else:
            self.power_state.setPixmap(self.px_power_red)
        
        if status.error_code < 0:
            self.error_state.setPixmap(self.px_error_red)
        else :
            self.error_state.setPixmap(self.px_error_green)
        
        frames = status.header.frame_id.split('/')[1:]
        
        if len(frames) == 3:
            self.tool_frame.setText("%s (%s)"%(frames[1],frames[2]))
            self.base_frame.setText(frames[0])
    
    def _robot_status_timeout_(self):
        
        self.communication_state.setPixmap(self.px_comm_red)
        self.power_state.setPixmap(self.px_power_red)
        self.error_state.setPixmap(self.px_error_grey)
        
        lng = self.getParent().getContext().getLanguage()
        
        self.tool_frame.setText(R.values.strings.unknown(lng))
        self.base_frame.setText(R.values.strings.unknown(lng))
        
    def _update_robot_io_(self, io):
        
        for i in range(0, len(io.digital_input_states)):
            getattr(self, 'digital_input_%i'%(io.digital_input_states[i].pin)).setPixmap(self._io_led[int(io.digital_input_states[i].state)])
            
        for i in range(0, len(io.digital_output_states)):
           getattr(self, 'digital_output_%i'%(io.digital_output_states[i].pin)).setPixmap(self._io_led[int(io.digital_output_states[i].state)])
        
    def onDestroy(self):
        self._robot_status_sub.shutdown()
        self._robot_io_sub.shutdown()
    
# my_unittest.shutdown()