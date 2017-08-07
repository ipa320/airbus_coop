#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : navigation_controller.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy
import time
import threading
import copy

from geometry_msgs.msg  import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseGoal

class NavigationController:
    
    # Methode to configure the controller
    TYPE_SIMPLE_GOAL = 1
    TYPE_LAZER_SACN_MATCHER = 2
    
    # Move base status
    PENDING         = 0
    ACTIVE          = 1
    PREEMPTED       = 2
    SUCCEEDED       = 3
    ABORTED         = 4
    REJECTED        = 5
    PREEMPTING      = 6
    RECALLING       = 7
    RECALLED        = 8
    LOST            = 9
    TIMEOUT         = 10
    
    # Errors codes
    ERR_CODE = [PREEMPTED,
                ABORTED,
                REJECTED,
                PREEMPTING,
                RECALLING,
                RECALLED,
                LOST,
                TIMEOUT]
    
    def __init__(self, move_type, preempt_requested_cb):
        
        self._preempt_requested_cb = preempt_requested_cb
        self._seq = 0
        
        pub_name  = ''
        sub_name  = ''
        
        if move_type == self.TYPE_LAZER_SACN_MATCHER:
            pub_name  = '/med_mesh_nav/goal'
            sub_name  = '/med_mesh_nav/result'
        else:
            pub_name  = '/move_base_simple/goal'
            sub_name  = '/move_base/result'
        
        self._mb_status = MoveBaseActionResult()
        self._mb_pub = rospy.Publisher(pub_name,
                                       PoseStamped, latch=True)
        
        self._mb_sub = rospy.Subscriber(sub_name,
                                        MoveBaseActionResult,
                                        self._mb_status_cb)
        
        self._wait_for_available_connections([self._mb_pub,
                                              self._mb_sub],
                                             rospy.Duration(5))
        
    def _wait_for_available_connections(self,
                      connections = [],
                      timeout=rospy.Duration(1)):
        
        r = rospy.Rate(10)
        initial_time = rospy.get_rostime()
        
        r.sleep()
        
        while not rospy.is_shutdown():
            if (rospy.get_rostime() - initial_time) > timeout:
                rospy.logerr(self.__class__.__name__+ \
                             " checks topics connection -> Timeout reached!")
                return False
            available = 1
            for conn in connections:
                available *= (int)(conn.impl.has_connections())
            if available:
                rospy.loginfo(self.__class__.__name__+ \
                             " all topics are connected.")
                return True
            else:
                r.sleep()
        return False
        
    def _mb_status_cb(self, msg):
        self._mb_status = msg
        
    def wait_for(self, condition, timeout):
        
        r = rospy.Rate(100)
        initial_time = rospy.get_rostime()
        
        self._mb_status = MoveBaseActionResult()

        while not rospy.is_shutdown():
        #{
            if self._preempt_requested_cb():
                return self.PREEMPTED
            elif (rospy.get_rostime() - initial_time) > timeout:
                return self.TIMEOUT
            elif self._mb_status.status.status != self.PENDING:
                if self._mb_status.status.status == condition:
                    return self.SUCCEEDED
                elif self._mb_status.status.status == self.ACTIVE:
                    pass
                elif self._mb_status.status.status in self.ERR_CODE:
                    return self._mb_status.status.status
                else:
                    pass
            else:
                pass
            
            r.sleep()
        #}End while
        
    def move_pose_command(self, goal, timeout):
        
        if not isinstance(goal, PoseStamped):
            self.logerr('MoveBaseController::move_pose_command  bad data type from goal!')
            return self.REJECTED
        
        goal.header.stamp = rospy.get_rostime()
        goal.header.seq   = self._seq
        goal.header.frame_id = "map"
        self._seq += 1
        
        try:
            self._mb_pub.publish(goal)
        except Exception as e:
            self.logerr('MoveBaseController::move_pose_command raise with exception "%s"'%e)
            return self.ABORTED
        
        result = self.wait_for(self.SUCCEEDED, timeout)
        return result
    
    def move_command(self, x, y, z, dz, dw, timeout):
        
        goal = PoseStamped()
        goal.header.frame_id    = "/map"
        goal.pose.position.x    = x
        goal.pose.position.y    = y
        goal.pose.orientation.z = dz
        goal.pose.orientation.w = dw
        
        return self.move_pose_command(goal, timeout)
        
    
#End of file
