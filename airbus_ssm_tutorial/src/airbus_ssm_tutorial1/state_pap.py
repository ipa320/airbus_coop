#!/usr/bin/env python

import sys
import rospy
import tf
import numpy
from airbus_ssm_core import ssm_state
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class RobotInfo(ssm_state.ssmState):
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],
									  io_keys=["group"])
		
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()

		
	def execution(self,ud):
		self.group = moveit_commander.MoveGroupCommander(ud.group)
		#Displays informations about the robot configuration
		print("============ Reference frame: %s" % self.group.get_planning_frame())
		print("============ End effector: %s" % self.group.get_end_effector_link())
		print("============ Position of End Effector: %s" % self.group.get_current_pose())
		print("============ Robot Groups:")
		print(self.robot.get_group_names())
		print("============ Printing robot state")
		print(self.robot.get_current_state())
		print("============")
		return "success"


class MoveCart(ssm_state.ssmState):
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],
									  io_keys=["target","group","step"])
									  
		#Rospy commander initialization
		#moveit_commander.roscpp_initialize(sys.argv)
		
		#Initialization of the robot commander
		self.robot = moveit_commander.RobotCommander()

		
	def acquireTargetUD(self,ud):
		target_string = ud
		target_list = target_string.split(" ")
		return target_list  
		

	def execution(self,ud):
		print("MOVE CARTESIAN")
		#self.listener = tf.TransformListener()
		
		#Lauch a robot config to get the group
		self.group = moveit_commander.MoveGroupCommander(ud.group)
		#Show target like this ['pos.x','pos.y','pos.z','roll','pitch','yaw'] 
		print(self.acquireTargetUD(ud.target))
		
		#Movement planification
		self.group.clear_pose_targets()
		
		self.pose_target = geometry_msgs.msg.Pose()
		#Translations
		self.pose_target.position.x = float(self.acquireTargetUD(ud.target)[0])
		self.pose_target.position.y = float(self.acquireTargetUD(ud.target)[1])
		self.pose_target.position.z = float(self.acquireTargetUD(ud.target)[2])
		
		#Rotations		
		self.roll = numpy.deg2rad(float(self.acquireTargetUD(ud.target)[3]))
		self.pitch = numpy.deg2rad(float(self.acquireTargetUD(ud.target)[4]))
		self.yaw = numpy.deg2rad(float(self.acquireTargetUD(ud.target)[5]))
		self.quaternion = tf.transformations.quaternion_from_euler(self.roll,self.pitch,self.yaw) 		
		self.pose_target.orientation.x = self.quaternion[0]
		self.pose_target.orientation.y = self.quaternion[1]
		self.pose_target.orientation.z = self.quaternion[2]
		self.pose_target.orientation.w = self.quaternion[3]
		
		self.group.set_pose_target(self.pose_target)
		self.plan = self.group.plan()

		rospy.sleep(2)
		self.group.execute(self.plan)

		#print("End effector real position")
		#print(self.listener.lookupTransform("/world", "/J6", rospy.Time(0)))
		
		#ud.step = float(ud.step) +1
		
		return "success"
		

class MoveArti(ssm_state.ssmState):
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],
									  io_keys=["joints","group","step"])
	
	def acquireJointsUD(self,ud):
		joints_string = ud
		joints_list = joints_string.split(" ")
		for i in range (0,6) :
			joints_list[i] = numpy.deg2rad(float(joints_list[i]))		
		return joints_list  

	def execution(self,ud):
		#rospy.sleep(20)
		
		print("MOVE ARTICULAR")
		self.group = moveit_commander.MoveGroupCommander(ud.group)
				
		print("Joint target")
		print(self.acquireJointsUD(ud.joints))
		
		self.group.set_joint_value_target(self.acquireJointsUD(ud.joints))
		self.plan = self.group.plan()
				
		rospy.sleep(2)
		self.group.execute(self.plan)

		#ud.step = float(ud.step) +1
		
		return "success"
	
		
		
		
class Grip(ssm_state.ssmState):
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],
									  io_keys=["tool","obj"])
									  
		#Rospy commander initialization
		#moveit_commander.roscpp_initialize(sys.argv)
		
		#Creation of the robot, the scene and the group interface with the world
		self.robot = moveit_commander.RobotCommander()

	def execution(self,ud):
		#Get the gripper movegroup
		self.group_gripper = moveit_commander.MoveGroupCommander(ud.tool)

		#Grip planification
		self.group_gripper.attach_object(ud.obj)
		#self.group_gripper.pick(ud.obj)
		rospy.sleep(1)	 	
		return "success"

class UnGrip(ssm_state.ssmState):
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],
									  io_keys=["tool","obj"])
									  
		#Rospy commander initialization
		#moveit_commander.roscpp_initialize(sys.argv)
		
		#Creation of the robot, the scene and the group interface with the world
		self.robot = moveit_commander.RobotCommander()

	def execution(self,ud):
		#Get the gripper movegroup
		self.group_gripper = moveit_commander.MoveGroupCommander(ud.tool)

		#Grip planification
		self.group_gripper.detach_object(ud.obj)
		#self.group_gripper.place(ud.obj)
		rospy.sleep(1)	 	
		return "success"


class GenEnvironment(ssm_state.ssmState):
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"])
	
		#moveit_commander.roscpp_initialize(sys.argv)
		
		self.scene = moveit_commander.PlanningSceneInterface()
		
	def createPose(self,x,y,z):
		self.p = PoseStamped()
		self.p.header.frame_id = "/world"
		self.p.pose.position.x = x
		self.p.pose.position.y = y
		self.p.pose.position.z = z
		self.p.pose.orientation.x = 0
		self.p.pose.orientation.y = 0
		self.p.pose.orientation.z = 0
		self.p.pose.orientation.w = 1
		return self.p
		
	def execution(self,ud):		
		self.scene.add_box("ground", self.createPose(0,0,-0.01), (10, 10, 0.02))
		self.scene.add_box("table1", self.createPose(0.5,0,0.25), (0.45, 0.45, 0.5))
		self.scene.add_box("table2", self.createPose(0,0.5,0.25), (0.45, 0.45, 0.5))
		self.scene.add_box("object", self.createPose(0.4,0,0.55), (0.1,0.1,0.1))
		#self.scene.add_box("object", self.createPose(0.4,0.3,0.55), (0.1,0.1,0.1))
		
		return "success"


class Commander(ssm_state.ssmState):
	#Class that will provide all the target/joint target to reach for each move class
	#in function of the box starting place and the box target place
	#This place should be acquired by the camera 
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success","relaunch"],io_keys=["joints","target","step"])

	def acquireObjectPos(self,obj):
		obj_string = obj
		obj_list = obj_string.split(" ")
		for i in range (0,6) :
			obj_list[i] = float(obj_list[i])
		
		for i in range (3,6) :
			obj_list[i] = numpy.deg2rad(obj_list[i])	
				
		return obj_list 
		
	def execution(self,ud):
		
		self.object_name = raw_input("Object name : ")
		self.object_pos_raw = raw_input("Object position : ")
		
		self.object_pos = self.acquireObjectPos(self.object_pos_raw)
		
		
		print("Object name = %s" %self.object_name)
		print ("Object position = %s" %self.object_pos)
		
		self.joint_1_rot = numpy.rad2deg(numpy.arctan(float(self.object_pos[1])/float(self.object_pos[0])))
		print (self.joint_1_rot)
		
		self.step = int(ud.step)
		
		if (self.step == 0):
			ud.joints = "%f 0 0 0 0 0" % numpy.float(self.joint_1_rot)
			rospy.sleep(20)
			return "relaunch"
		
		elif (self.step == 1):
			ud.joints = "%f 20 20 0 0 0" % numpy.float(self.joint_1_rot)
			return "relaunch"
			
		elif (self.step > 1):
			return "success"
			
		else :
			print("YOUR CODE IS NOT WORKING LIKE YOU WOULD LIKE")
			return "success"
			
	  
		
	
