#!/usr/bin/env python
#
# Copyright 2015 Airbus
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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
import ssm_scxml_interpreter
import ssm_introspection

import actionlib
import ssm_core.msg
from std_msgs.msg import Empty, Bool 

class ssmActionServer(object):
    _feedback = ssm_core.msg.SSMFeedback()
    _result = ssm_core.msg.SSMResult()
    
    def __init__(self):
        
        ##SSM variables
        self._SSM = None
        self._introspection = None
        self._server_name = rospy.get_param('~/ssm_server_name', '/ssm')
        self._preempt = False
        self._running = False

        self._as = actionlib.SimpleActionServer(self._server_name, ssm_core.msg.SSMAction,execute_cb=self.execute_cb, auto_start=False)
        self._as.register_preempt_callback(self.as_preempt_cb)
        self._as.start()
        
        rospy.loginfo("SSM Action Server is now ready !")
        
    def as_preempt_cb(self):
        self._SSM.request_preempt()
  
    def readSCXML(self, file):
        file_ = ssm_scxml_interpreter.get_pkg_dir_from_prefix(file)
        if(os.path.isfile(file_) == False):
            ##test file existence if only the name of the file was given (suppose to be in the resources dir of ssm_core
            path = "${ssm_core}/resources/"+file_+".scxml"
            file_ = ssm_scxml_interpreter.get_pkg_dir_from_prefix(path)
            if(os.path.isfile(file_) == False):
                rospy.logerr("[SSM] %s not found. Either give only the name of file without the scxml extension and put it in the resource folder.\n Or give the full path (${pkg}/dir/file.scxml)" 
                             %rospy.get_param('/ssm_node/scxml_file'))
                return False
            
        try:
            interpreter = ssm_scxml_interpreter.ssmInterpreter(file_)
            self._SSM = interpreter.convertSCXML()
        except Exception as e:  
            rospy.logerr("[SSM] error during interpretation of the SCXML ")
            rospy.logerr(e)
            self._SSM = None
            return False
        try:
            self._SSM.check_consistency()
        except Exception as e:  
            rospy.logerr("[SSM] error during consistency checks.")
            rospy.logerr(e)
            self._SSM = None
            return False

        self._introspection = ssm_introspection.ssmIntrospection(self._SSM, self._as)
        self._introspection.start()
        rospy.loginfo("[SSM] : %s file loaded and created." %file)
        return True      
    
        
        return True
    
    def runSSM(self):
        if(self._SSM is not None):
            try:
                outcome = self._SSM.execute()
            except Exception as e:
                rospy.logerr(e)
                self._introspection.stop()
                self._running = False
                self._SSM = None
                outcome = "aborted"
            self._introspection.stop()

        self._SSM = None
        self._running = False
        return outcome
        
    def execute_cb(self, file):
        self._SSM = None
        self._feedback.current_active_states = "no one"
        self._result.outcome = "aborted"
        #loading the file
        rospy.loginfo("[SSM ActionServer] Received a new request : %s"%file.scxml_file)
        result = self.readSCXML(file.scxml_file)
        if(result == False):
            self._as.set_aborted(self._result,text="Error while loading the file : %s"%file.scxml_file)
            return
        
        rospy.loginfo("[SSM ActionServer] %s file loaded and state machine created. Will start the execution." %file.scxml_file)
        self._result.outcome = self.runSSM() 
        if(self._result.outcome == 'aborted'):
            self._as.set_aborted(self._result,text="Error during the execution.")
        if(self._result.outcome == 'preempt'):
            self._as.set_preempted(self._result,text="State Machine preempted.")
        else:
            self._as.set_succeeded(self._result,"SSM has been executed without error.")
        

    
    
        
    

        
