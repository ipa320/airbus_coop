#!/usr/bin/env python

import rospy
from ssm_core import ssm_state
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

class WaitForEffort(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                   outcomes = ["success", "fail"], io_keys=[])
        
        self.jointstates_sub = None
        self.actualTorque = []
        self.execution = False
        self.torqueAcquired = False
        self.torqueReached = False
        
    def _jointstates_cb(self, js):
        
        if(self.execution == True):
            if(self.torqueAcquired == False):
                self.actualTorque = js.effort
                self.torqueAcquired = True
            else:
                torqueOver = False
                for i_jt in range(len(js.effort)):
                    if(abs(self.actualTorque[i_jt] - js.effort[i_jt])>0.6):
                        torqueOver = True
                
                if(torqueOver):
                    self.torqueReached = True
                
    
    def execution(self, ud):
        self.jointstates_sub = rospy.Subscriber("/lwr/joint_states",JointState, self._jointstates_cb)
        self.actualTorque = []
        self.torqueAcquired = False
        self.torqueReached = False
        self.execution = True
        cpt = 0
        r = rospy.Rate(10)
        while (not((self.preempt_requested()) or (self.torqueReached))):
            if (cpt >6000):
                rospy.logerr("[Hand Lwr Skill] : Wait for Effort time out")
                self.execution = False
                return "fail"
            else:
                cpt = cpt+1
                r.sleep()
        
        self.execution = False
        rospy.loginfo("[Hand Lwr Skill] : Effort Detected")
        return "success"
    


class TurnGravityOn(ssm_state.ssmState):
    
    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["success"])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
        
    def execute(self, ud):
        
        self.pub_rsi_prog_cmd.publish(50)
        rospy.sleep(2)
        self.pub_rsi_prog_cmd.publish(2)
        return "success"
    
class TurnGravityOff(ssm_state.ssmState):
    
    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["success"])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
        
    def execution(self, ud):
        
        self.pub_rsi_prog_cmd.publish(51)
        rospy.sleep(2)
        self.pub_rsi_prog_cmd.publish(2)
        return "success"
    
class TurnCollisionOn(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["success"])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
        
    def execution(self, ud):
        
        self.pub_rsi_prog_cmd.publish(52)
        rospy.sleep(2)
        self.pub_rsi_prog_cmd.publish(2)
        return "success"
    
class TurnCollisionOff(ssm_state.ssmState):
    
    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["success"])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
        
    def execution(self, ud):
        
        self.pub_rsi_prog_cmd.publish(53)
        rospy.sleep(2)
        self.pub_rsi_prog_cmd.publish(2)
        return "success"
        
