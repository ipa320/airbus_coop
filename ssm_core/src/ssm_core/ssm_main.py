#!/usr/bin/env python

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : Interpreter.py
# Authors : Ludovic DELVAL
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Ludovic DELVAL <ludovic.delval.external@airbus.com>
#
#
#
#
################################################################################


import ssm_scxml_interpreter
import ssm_introspection
import ssm_core.srv
from std_msgs.msg import Int8, Empty, Bool

import rospy
import sys
import smach_ros

class ssmMain:
    
    def __init__(self):
        self._SSM = None
        self._introspection = None
        self._server_name = rospy.get_param('ssm_server_name', '/ssm')
        self._init_srv = rospy.Service(self._server_name + '/srv/init',ssm_core.srv.SSM_init, self._init_SSM_srv)
        self._start_sub = rospy.Subscriber(self._server_name + '/start',Empty,self.start, queue_size=1)
        self._init_sub = rospy.Subscriber(self._server_name + '/init', Empty, self._init_SSM_cb, queue_size=1)
        self._preempt_sub = rospy.Subscriber(self._server_name + '/preempt',Empty, self._preempt_cb, queue_size=1)
        self._pause_sub = rospy.Subscriber(self._server_name + '/pause', Bool, self._pause_cb, queue_size=1)
        self._status_pub = rospy.Publisher(self._server_name + '/status', Int8, queue_size=1)
        self._preempt = False
        self._loading = False
        rospy.loginfo("SSM is now ready !")
        
    def _init_SSM_cb(self, msg):
        if(self._loading == False):
            self._loading = True
            self._preempt = False
            self._init_SSM()
            
    def _init_SSM_srv(self, msg):
        rospy.set_param('scxml_file', str(msg.file_scxml.data))
        response = Bool
        response.data = False
        if(self._loading == False):
            self._loading = True
            self._preempt = False
            response.data = self._init_SSM()
             
        return response
        
    def _init_SSM(self):
        result = self.readSCXML()
        if(result):
            self._status_pub.publish(1)
        else:
            self._status_pub.publish(-10)
        self._loading = False
        return result
        
    def readSCXML(self):
        file = ssm_scxml_interpreter.get_pkg_dir_from_prefix(rospy.get_param('/ssm_core/scxml_file', default=("${ssm_core}/resources/default.scxml")))
        try:
            interpreter = ssm_scxml_interpreter.ssmInterpreter(file)
            self._SSM = interpreter.readFile()
            self._SSM.check_consistency()
            self._introspection = ssm_introspection.ssmIntrospection(self._SSM)
            self._introspection.start()
        except Exception as e:
            rospy.logerr(e)
            self._SSM = None
            return False
        rospy.loginfo("[SSM] : %s file loaded and created." %file)
        return True      
    
    def start(self, msg):
        if(self._SSM is not None):
            self._status_pub.publish(2)
            try:
                self._SSM.execute()
            except Exception as e:
                self._status_pub.publish(-2)
                rospy.logerr(e)
                self._introspection.stop()
                return
            if(self._preempt == False):
                self._status_pub.publish(10)
                rospy.loginfo("[SSM] : Finished without error")
            self._introspection.stop()
        else:
            rospy.logwarn("[SSM] : Start requested but there is no state machine loaded !")
            
    def _preempt_cb(self, msg):
        self._preempt = True
        self._status_pub.publish(-2)
        
    def _pause_cb(self, msg):
        if(msg.data):
            self._status_pub.publish(-1)
        else:
            self._status_pub.publish(2)
    
    
    
        
    

        
