#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
# Copyright 2015 Airbus
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
import os
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

class QAgiSubscriber(QObject):
    
    CALLBACK_SIGN = 'callbackTriggered'
    TIMEOUT_SIGN  = 'timeoutTriggered'
    
    def __init__(self,
                 parent,                   # Main thread (QObject)
                 name,                     # Topic name
                 data_class,               # Data class ROS message
                 cb_slot      = None,      # Callback Qt slot triggered by ROS message 
                 timeout_slot = None,      # Timeout Qt slot triggered when unreceived message on time
                 timeout      = None,      # Timeout duration
                 auto_start   = True,      # Auto start connection
                 max_rate     = None):     # Limitation frequency to emit SIGNAL to call cb_slot
        
        QObject.__init__(self)
        
        self._parent        = parent
        self._sub           = None
        self._topic_name    = name
        self._data_class    = data_class
        self._cb_slot       = cb_slot
        self._timeout       = timeout
        self._timeout_slot  = timeout_slot
        self._timeout_t     = None
        
        self._last_call_time    = rospy.get_rostime()
        self._min_time_dur_b2c  = None # Minimum time duration between 2 call
        
        if isinstance(max_rate, rospy.Rate):
            self._min_time_dur_b2c = max_rate.sleep_dur
            self._cb_ptr = self._inhibited_callback
        elif isinstance(max_rate, int):
            self._min_time_dur_b2c = rospy.Rate(max_rate).sleep_dur
            self._cb_ptr = self._inhibited_callback
        else:
            self._cb_ptr = self._callback
        
        self._parent.connect(self, SIGNAL(self.CALLBACK_SIGN), self._cb_slot)
        
        if self._timeout_slot:
            self._parent.connect(self, SIGNAL(self.TIMEOUT_SIGN), self._timeout_slot)
            
        if auto_start:
            self.start()
            
    def remaining(self):
        elapsed = rospy.get_rostime() - self._last_call_time
        return elapsed
    
    def _inhibited_callback(self, msg):
        """Callback with max rate limitation
        Check time between two call and inhibit signal if diff time is too small
        If (T-1 - T) < Tmax_rate -> inhibit slot
        else call slot
        """
        
        if self.remaining() >= self._min_time_dur_b2c:
            self._callback(msg)
    
    def _callback(self, msg):
        """Callback without max rate inhibition"""
        self.emit(SIGNAL(self.CALLBACK_SIGN), msg)
        self._last_call_time = rospy.get_rostime()
        
    def _slot_check_timeout(self):
        
        if self.remaining() > self._timeout:
            self.emit(SIGNAL(self.TIMEOUT_SIGN))
    
    def setTimeout(self, timeout):
        self._timeout = timeout
        
    def start(self):
        
        self._sub = rospy.Subscriber(self._topic_name,
                                     self._data_class,
                                     self._cb_ptr)
        
        if self._timeout:
            self._timeout_t = QTimer(self)
            self.connect(self._timeout_t, SIGNAL("timeout()"), self._slot_check_timeout)
            self._timeout_t.start(self._timeout.to_nsec()*(1e-6)/3)
            
        self._last_call_time = rospy.get_rostime()
        
    subscribe = start
    
    def shutdown(self):
        
        if self._timeout is not None:
            self._timeout_t.stop()
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None
        
    unregister = shutdown
    stop = shutdown
    
