#!/usr/bin/env python

import rospy
from ssm_core import ssm_state
import common_lib.pt_xml
from common_lib.pt_xml import read_cartesian_from_xml, read_joint_from_xml
from xml.etree import ElementTree
from std_msgs.msg import Int8, Bool, Empty
from geometry_msgs.msg  import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.linalg.linalg_version import linalg_version
from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix
        

class MoveArticularFromXML(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["finish"],
                                    io_keys = ['trajectory_file' , 'joint_id', 'joint_dof','movement_timeout', 'arm_prefix'])
        
        self.pub_joint_cmd = None
        self.pub_stop_cmd = None
        
        self.sub_trajectory_finished = None
        self.finished_ = False
        
    def _finished_cb(self, finished):
            self.finished_ = True
        
    def read_point(self, ud):
        ##Process data
        xml = ElementTree.parse(get_pkg_dir_from_prefix(str(ud.trajectory_file)))
        root = xml.getroot()
        dof_ = int(ud.joint_dof)
        return read_joint_from_xml(root,ud.joint_id,dof_)
        
    def move(self, traj, ud):
        self.sub_trajectory_finished = rospy.Subscriber("/"+ud.arm_prefix+"/movement_finished",Empty, self._finished_cb)
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
            #if ((rospy.Time.now() - now_) > timeout_):
            #    rospy.logerr("[Arm Skill] : Articular Command Timed out")
            #    self.stop_cmd_pub.publish()
            #    self.execution = False
            #    return "preempt"
            #else:
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
        rospy.sleep(0.1)
        pt = self.read_point(ud)
        if(pt == None):
            return "preempt"
        return self.move(pt, ud)


class MoveCartesianFromXML(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                    outcomes = ["finish"],
                                    io_keys = ['trajectory_file' , 'point_id', 'movement_timeout', 'arm_prefix'])
        
        self.pub_cart_cmd = None
        self.pub_stop_cmd = None
        
        self.sub_trajectory_finished = None
        self.finished_ = False
        
    def _finished_cb(self, finished):
            self.finished_ = True
        
    def read_point(self, ud):
        ##Process data
        xml = ElementTree.parse(get_pkg_dir_from_prefix(str(ud.trajectory_file)))
        root = xml.getroot()
        pt_, tool_  = read_cartesian_from_xml(root,ud.point_id)
        return pt_
        
    def move(self, pt, ud):
        self.sub_trajectory_finished = rospy.Subscriber("/"+ud.arm_prefix+"/movement_finished",Empty, self._finished_cb)
        self.finished_ = False     
        pt.pose.position.x = pt.pose.position.x/1000.0
        pt.pose.position.y = pt.pose.position.y/1000.0
        pt.pose.position.z = pt.pose.position.z/1000.0
        r = rospy.Rate(10)
        trajectory_done = False
        timeout_ = int(ud.movement_timeout)
        if(timeout_ < 10):
            rospy.logwarn("[Arm Skill] : Cartesian Command Timeout set to 10 seconds !")
            timeout_ = 10
            
        now_ = rospy.Time.now()
        self.pub_cart_cmd.publish(pt)
        rospy.loginfo("[Arm Skill] : Moving")
        while (not((self.preempt_requested()) or (self.finished_))):
            #if ((rospy.Time.now() - now_) > timeout_):
            #    rospy.logerr("[Arm Skill] : Cartesian Command Timed out")
            #    self.stop_cmd_pub.publish()
            #    self.execution = False
            #    return "preempt"
            #else:
                r.sleep()
                
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
        pt = self.read_point(ud)
        if(pt == None):
            return "preempt"
        return self.move(pt, ud)
    
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
            #if ((rospy.Time.now() - now_) > timeout_):
            #    rospy.logerr("[Arm Skill] : Cartesian Command Timed out")
            #    self.stop_cmd_pub.publish()
            #    self.execution = False
            #    return "preempt"
            #else:
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

class WaitForEffort(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,
                                   outcomes = ["success", "fail"], io_keys=[])
        
        self.jointstates_sub = None
        self.actualTorque = []
        self.execution_ = False
        self.torqueAcquired = False
        self.torqueReached = False
        
    def _jointstates_cb(self, js):
        if(self.execution_ == True):
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
        self.actualTorque = []
        self.torqueAcquired = False
        self.torqueReached = False
        self.execution_ = True
        self.jointstates_sub = rospy.Subscriber("/lwr/joint_states",JointState, self._jointstates_cb)
        cpt = 0
        r = rospy.Rate(10)
        while (not(self.torqueReached)):
            if (cpt >6000):
                rospy.logerr("[Hand Lwr Skill] : Wait for Effort time out")
                self.execution_ = False
                return "fail"
            else:
                cpt = cpt+1
                r.sleep()
        
        self.execution_ = False
        rospy.loginfo("[Hand Lwr Skill] : Effort Detected")
        return "success"

'''    
class TurnGravityOn(ssm_core.AirState):
    
    def __init__(self):
        ssm_core.AirState.__init__(self,
                                    io_keys = ['outcome' , 'data'])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
        
    def execute(self, ud):
        
        self.pub_rsi_prog_cmd.publish(50)
        rospy.sleep(2)
        self.pub_rsi_prog_cmd.publish(2)
        ud.outcome = "succeeded"
        return self.SUCCEEDED
    
class TurnGravityOff(ssm_core.AirState):
    
    def __init__(self):
        ssm_core.AirState.__init__(self,
                                    io_keys = ['outcome' , 'data'])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
    def execute(self, ud):
        
        self.pub_rsi_prog_cmd.publish(51)
        rospy.sleep(2)
        self.pub_rsi_prog_cmd.publish(2)
        ud.outcome = "succeeded"
        return self.SUCCEEDED
    
class TurnCollisionOff(ssm_core.AirState):
    
    def __init__(self):
        ssm_core.AirState.__init__(self,
                                    io_keys = ['outcome' , 'data'])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
    def execute(self, ud):
        self.Execution = True
        self.pub_rsi_prog_cmd.publish(53) ##SET CONTACT DETECTION ON
        rospy.sleep(2.0)
        self.pub_rsi_prog_cmd.publish(2)
        ud.outcome = "succeeded"
        return self.SUCCEEDED
        
class TurnCollisionOn(ssm_core.AirState):

    def __init__(self):
        ssm_core.AirState.__init__(self,
                                    io_keys = ['outcome' , 'data'])
        
        self.pub_rsi_prog_cmd = rospy.Publisher('/lwr/prog_cmd',Int8)
        
    def execute(self, ud):
        
        self.Execution = True
        self.pub_rsi_prog_cmd.publish(52) ##SET CONTACT DETECTION ON
        rospy.sleep(2.0)
        self.pub_rsi_prog_cmd.publish(2)
        ud.outcome = "succeeded"
        return self.SUCCEEDED

class MoveToObject(ssm_core.AirState):
    
    
    def __init__(self):
        ssm_core.AirState.__init__(self,
                                      io_keys = ['outcome' , 'data'])
        self.pose_cmd_pub = rospy.Publisher("/lwr/cartesian_cmd",PoseStamped,queue_size=1)
        self.stop_cmd_pub = rospy.Publisher("/lwr/stop_cmd",Empty,queue_size=1)
        
        self.trajectory_finished_sub = rospy.Subscriber("/lwr/movement_finished",Empty, self._finished_cb)
        self.execution = False
        self.finished_ = False
        
        
    def _finished_cb(self, finished):
        if(self.execution == True):
            self.finished_ = True
        
    def execute(self, ud):
       
        self.finished_ = False     
        rospy.loginfo("[RobotArm] : Moving")
        Pose_exe =  ud.data['TargetPose']
        if(Pose_exe == None):
            rospy.logerr("[LWR Arm Skill] : Computed Point Error")
            return self.ABORTED
        r = rospy.Rate(10)
        cpt=0
        trajectory_done = False
        self.execution = True
        self.pose_cmd_pub.publish(Pose_exe)
        
        while (not((self.preempt_requested()) or (self.finished_))):
            if (cpt >6000):
                rospy.logerr("[LWR Arm Skill] : Cartesian Command Timed out")
                self.stop_cmd_pub.publish()
                self.execution = False
                return self.ABORTED
            else:
                cpt= cpt+1
                r.sleep()
                
        self.execution = False
        if(self.finished_):
            rospy.loginfo("[LWR Arm Skill] : Clearing Trajectory")
            ud.data['TargetPose'] = None
            ud.outcome="succeeded"
            return self.SUCCEEDED
        else:
            rospy.logerr("[LWR Arm Skill] : Command Preempted")
            self.stop_cmd_pub.publish()
            return self.ABORTED
'''



#End of file

