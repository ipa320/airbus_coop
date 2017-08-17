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

from ssm_core import ssm_state
from std_msgs.msg import Empty
from geometry_msgs.msg  import PoseStamped
        
    
class MoveArticular(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["finish"],
                                    io_keys = ['target_joint','movement_timeout', 'arm_prefix'])
        
        self.pub_joint_cmd = None
        self.pub_stop_cmd = None
        
        self.sub_trajectory_finished = None
        self.finished_ = False
        
    def _finished_cb(self, finished):
            self.finished_ = True
        
    def move(self, traj, ud):
        self.sub_trajectory_finished = rospy.Subscriber("/"+arm_prefix+"/movement_finished",Empty, self._finished_cb)
        self.finished_ = False     
 
    
        if(len(traj.positions) == 0):
            rospy.logerr("[Arm Skill] : Reading Path Error")
            return "preempt"
        
        r = rospy.Rate(10)
        trajectory_done = False
        timeout_ = int(ud.movement_timeout)
        if(timeout_ < 10):
            rospy.logwarn("[Arm Skill] : Articular Command Timeout set to 10 seconds !")
            timeout_ = 10
            
        now_ = rospy.Time.now()
        self.pub_joint_cmd.publish(traj)
        rospy.loginfo("[Arm Skill] : Moving")
        while (not((self.preempt_requested()) or (self.finished_))):
            if ((rospy.Time.now() - now_) > timeout_):
                rospy.logerr("[Arm Skill] : Articular Command Timed out")
                self.stop_cmd_pub.publish()
                self.execution = False
                return "preempt"
            else:
                r.sleep()
                
        if(self.finished_):
            return "finish"
        else:
            rospy.logerr("[Arm Skill] : Command Preempted")
            self.stop_cmd_pub.publish()
            return "preempt"    
        
    def execution(self, ud):
        self.pub_joint_cmd = rospy.Publisher("/"+ud.arm_prefix+"/articular_cmd",JointTrajectoryPoint,queue_size=1)
        self.pub_stop_cmd = rospy.Publisher("/"+ud.arm_prefix+"/stop_movement",Empty,queue_size=1)
        joint = ud.target_joint
        rospy.sleep(1)
        if(joint == None):
            return "preempt"
        return self.move(joint, ud)
    
class MoveCartesian(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["finish"],
                                    io_keys = ['target_point' , 'tool', 'movement_timeout', 'arm_prefix'])
        

        
        self.sub_trajectory_finished = None
        self.finished_ = False
        
    def _finished_cb(self, finished):
            self.finished_ = True
        
    def move(self, pt, ud):
        self.sub_trajectory_finished = rospy.Subscriber("/"+ud.arm_prefix+"/movement_finished",Empty, self._finished_cb)
        self.finished_ = False     

        trajectory_done = False
        timeout_ = int(ud.movement_timeout)
        if(timeout_ < 10):
            rospy.logwarn("[Arm Skill] : Cartesian Command Timeout set to 10 seconds !")
            timeout_ = 10
            
        now_ = rospy.Time.now()
        self.pub_cart_cmd.publish(pt)
        rospy.loginfo("[Arm Skill] : Moving")
        while (not((self.preempt_requested()) or (self.finished_))):
            if ((rospy.Time.now() - now_) > timeout_):
                rospy.logerr("[Arm Skill] : Cartesian Command Timed out")
                self.stop_cmd_pub.publish()
                self.execution = False
                return "preempt"
            else:
                rospy.sleep(0.1)
                
        if(self.finished_):
            return "finish"
        else:
            rospy.logerr("[Arm Skill] : Command Preempted")
            self.stop_cmd_pub.publish()
            return "preempt"    
        
    def execution(self, ud):
        self.pub_cart_cmd = rospy.Publisher("/"+ud.arm_prefix+"/cartesian_cmd",PoseStamped,queue_size=1)
        self.pub_stop_cmd = rospy.Publisher("/"+ud.arm_prefix+"/stop_movement",Empty,queue_size=1)
        rospy.sleep(1)
        if(ud.target_point == None):
            rospy.logerr("[Arm Skill] : Target Point Not Defined !")
            return "preempt"
        return self.move(ud.target_point, ud)



#End of file

