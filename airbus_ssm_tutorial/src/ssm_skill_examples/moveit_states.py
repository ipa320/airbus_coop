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

from airbus_ssm_core import ssm_state, ssm_state_machine

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from threading import Thread

import moveit_commander
import ast
import math
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal, ExecuteTrajectoryResult, JointConstraint, Constraints
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest, GetMotionPlan, GetMotionPlanRequest 

class MoveArticular(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=['success'], io_keys=['group', 'target', 'vel_factor', 'type'])
        self.group = None
        self.preempt_thread = None
        self.done = False
        
    def preempt_loop(self):
        while(self.done == False):
            if(self.preempt_requested()):
                try:
                    self.group.stop()
                except Exception as ex:
                    rospy.logerr(ex)
            rospy.sleep(0.01)

    def execution(self, ud):
        ##Get trajectory executor
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_max_velocity_scaling_factor(float(ud.vel_factor))
        self.group.set_start_state_to_current_state()
        self.done = False
        
        if isinstance(ud.target, str):
            target = ast.literal_eval(ud.target)
        elif isinstance(ud.target, list) or isinstance(ud.target, dict):
            target = ud.target
        elif isinstance(ud.target, JointState):
            target = ud.target
        else:
            rospy.logerr("Not compatible data 'target' with current value : %s"%str(ud.target))
            return "preempt"
        if(ud.type == "degree"):
            for elmt in target:
                target[target.index(elmt)]= math.radians(elmt)
        try:
            self.group.set_joint_value_target(target)
        except Exception as ex:
            rospy.logerr(ex)
            return "preempt"

        self.preempt_thread = Thread(target=self.preempt_loop)
        self.preempt_thread.start()
        success = self.group.go(target)
        self.done = True
        self.preempt_thread.join()
        if success == True:
            return "success"
        else:
            return "preempt"
        
        
class GenerateArticularPlan(ssm_state.ssmServiceState):
    def __init__(self):
        ssm_state.ssmServiceState.__init__(self, "/plan_kinematic_path", GetMotionPlan, outcomes=["success"], io_keys=["target", "group_name", "plan", "namespace"], timeout_ = 10, timeout_outcome = "preempt")
        
    def convert_to_joint_constraints(self, current_js, target, type):
        jcs = []
        if(isinstance(target, basestring)):
           target = ast.literal_eval(target)
        if isinstance(target, list):
            for i_jt in range(len(target)):
                jc = JointConstraint()
                jc.joint_name = current_js.name[i_jt]
                if(type == "degree"):
                    jc.position = math.radians(target[i_jt])
                else:
                    jc.position = target[i_jt]
                jc.tolerance_above = jc.tolerance_below = 0.001
                jc.weight = 1.0
                jcs.append(jc)
        elif isinstance(target, dict):
            for key in len(target):
                jc = JointConstraint()
                jc.joint_name = key
                if(type == "degree"):
                    jc.position = math.radians(target[key])
                else:
                    jc.position = target[key]
                jc.tolerance_above = jc.tolerance_below = 0.001
                jc.weight = 1.0
                jcs.append(jc)
        elif isinstance(target, JointState):
            for i_jt in range(len(target.name)):
                jc = JointConstraint()
                jc.joint_name = target.name[i_jt]
                jc.position = target.position[i_jt]
                jc.tolerance_above = jc.tolerance_below = 0.001
                jc.weight = 1.0
                jcs.append(jc)
        else:
            rospy.logerr("[GenerateArticularPlan] Not compatible data 'target' with current value : %s"%str(target))
            raise Exception()
        return jcs
            
    def get_last_joint_state(self, namespace):
        print(namespace)
        if(namespace != '""'):
            state = rospy.wait_for_message("/"+namespace+"/joint_states", JointState, timeout=5)
        else:
            state = rospy.wait_for_message("/joint_states", JointState, timeout=5)
        return state
                
    def setup_request(self, ud):
        request = GetMotionPlanRequest()
        request.motion_plan_request.group_name = ud.group_name
        current_state = self.get_last_joint_state(ud.namespace)
        jcs = self.convert_to_joint_constraints(current_state, ud.target, ud.type)
        if(len(jcs) == 0):
            raise Exception()
        request.motion_plan_request.start_state.joint_state = current_state
        request.motion_plan_request.start_state.is_diff = True
        _constraint = Constraints()
        _constraint.joint_constraints = jcs
        request.motion_plan_request.goal_constraints.append(_constraint)
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 1.0
        request.motion_plan_request.max_acceleration_scaling_factor = 1.0
        return request
    
    def analyse_answer(self, ud, service_answer):
        if(service_answer.motion_plan_response.error_code.val != 1):
            rospy.logerr("[GenerateCartesianPlan] Planning failed")
            return "preempt"
        plan = service_answer.motion_plan_response.trajectory
        ud.plan = plan
        return "success"

    
class GenerateCartesianPlan(ssm_state.ssmServiceState):
    def __init__(self):
        ssm_state.ssmServiceState.__init__(self, "/compute_cartesian_path", GetCartesianPath, outcomes=["success"], io_keys=["target", "group_name", "plan", "namespace"], timeout_ = 10, timeout_outcome = "preempt")
        
    def convert_to_waypoints(self, target):
        waypoints = []
        ##Convert from string
        if(isinstance(target, basestring)):
           target = ast.literal_eval(target)
        if(isinstance(target,list)):
            if(isinstance(target[0],list)): ##multiple points
               for pt in target:
                   waypoints.append(self.convertToPose(pt))
            else: ##only one point
                waypoints.append(self.convertToPose(target))
        elif(isinstance(target, Pose) or isinstance(target, PoseStamped)): # 1 points but in the Pose or Posestamped format
            waypoints.append(target)
        else:
            rospy.logerr("[GenerateCartesianPlan] Userdata 'target' with current value %s is not compatible." %str(target))
        return waypoints
    
    def convertToPose(self, pt):
        if(isinstance(pt,Pose) or isinstance(pt,PoseStamped)):
            return pt
        else:
            pose = Pose()
            if(len(pt) != 7):
                rospy.logerr("[GenerateCartesianPlan] Not well formated point in the userdata 'target' : %s "%str(pt))
                raise Exception()
            else:
                pose.position.x = pt[0]
                pose.position.y = pt[1]
                pose.position.z = pt[2]
                pose.orientation.x = pt[3]
                pose.orientation.y = pt[4]
                pose.orientation.z = pt[5]
                pose.orientation.w = pt[6]
            return pose
        
    def get_last_joint_state(self, namespace):
        if(namespace != '""'):
            state = rospy.wait_for_message("/"+namespace+"/joint_states", JointState, timeout=5)
        else:
            state = rospy.wait_for_message("/joint_states", JointState, timeout=5)
        return state
                
    def setup_request(self, ud):
        request = GetCartesianPathRequest()
        request.group_name = ud.group_name
        waypoints = self.convert_to_waypoints(ud.target)
        if(len(waypoints) == 0):
            raise Exception()
        request.waypoints = waypoints
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.joint_state = self.get_last_joint_state(ud.namespace)
        return request
    
    def analyse_answer(self, ud, service_answer):
        if(service_answer.error_code.val != 1):
            rospy.logerr("[GenerateCartesianPlan] Planning failed")
            return "preempt"
        plan = service_answer.solution
        fraction = service_answer.fraction
        if(fraction ==0):
            rospy.loginfo(plan)
            rospy.loginfo(fraction)
            rospy.logerr("[GenerateCartesianPlan] Planning fraction is different than 1")
            return "preempt"
        ud.plan = plan
        return "success"


class Move(ssm_state.ssmActionClientState):
    def __init__(self):
        ssm_state.ssmActionClientState.__init__(self, "/execute_trajectory", ExecuteTrajectoryAction , outcomes=["success"], io_keys=["plan"], timeout_server = 10, timeout_server_outcome = "preempt", timeout_action = 120, timeout_action_outcome = "preempt")
    
    def setup_goal(self, ud):
        goal = ExecuteTrajectoryGoal()
        goal.trajectory = ud.plan
        return goal
    
    def analyse_result(self, ud, result):
        if(result.error_code.val != 1):
            rospy.logerr("[Move] Moving failed")
            return "preempt"
        return "success"
    
class MoveCartesian(ssm_state_machine.ssmStateMachine):
    def __init__(self):
        ssm_state_machine.ssmStateMachine.__init__(self, input_keys=["target", "group_name", "namespace"], outcomes=["success"])
        
        with self:
            self.add('Planning', GenerateCartesianPlan(),
                     {'success' : 'Move'}, 
                     {"plan" : "plan", "target" : "target", "group_name" : "group_name", "namespace" : "namespace"})
            
            self.add('Move', Move(),
                     {'success' : 'success'}, 
                     {"plan" : "plan", "namespace" : "namespace"})
        
            
        
#End of file
