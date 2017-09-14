#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np
from airbus_ssm_core import ssm_state
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import ast


class InitMoveit(ssm_state.ssmState):
	'''@SSM
	Description : Init skill that initialize the moveit_commander
	User-data : N/A
	Outcome :
	- success : sucessfully initialized moveit_commander
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"])
				
	def execution(self,ud):
		#moveit_commander.roscpp_initialize(sys.argv)
		#self.robot = moveit_commander.RobotCommander()
		#self.scene = moveit_commander.PlanningSceneInterface()	
		rospy.sleep(1)
		print("Moveit commander initialized !") 
		return("success")



class RobotInfo(ssm_state.ssmState):
	'''@SSM
	Description :  Info skill that displays information about the used robot
	User-data : N/A
	Outcome :
	- success : sucessfully displayed information
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"])
		
	def execution(self,ud):
		self.robot = moveit_commander.RobotCommander()
		#Displays informations about the robot configuration
		print("============ Robot Groups:")
		print(self.robot.get_group_names())
		print("============ Printing robot state")
		print(self.robot.get_current_state())
		print("============")
		return "success"



class GroupInfo(ssm_state.ssmState):
	'''@SSM
	Description :  Info skill that displays information about a selected moveit move group
	User-data :
	- group : a name of a robot moveit move group (string)
	Outcome :
	- success : sucessfully displayed information
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],io_keys=["group"])

	def execution(self,ud):
		self.group = moveit_commander.MoveGroupCommander(ud.group)
		#Displays informations about the group configuration	
		print("============ Reference frame: %s" % self.group.get_planning_frame())
		print("============ End effector: %s" % self.group.get_end_effector_link())
		print("============ Position of End Effector: %s" % self.group.get_current_pose())
		return "success"



class MoveCart(ssm_state.ssmState):
	'''@SSM
	Description :  Action skill that makes a moveit move group move to a given target (cartesian) 
	User-data :
	- group : a moveit move group name, by default the end effector will try to match his 
	frame with the target frame (string)
	- target : a position and orientation of a target frame (['pos.x','pos.y','pos.z','roll','pitch','yaw'])
			  (it could be a list of target : [[target1],[target2],[target3]])
	- frame : the name of the reference frame (string)
	- offset : an offset for the target (['pos.x','pos.y','pos.z','roll','pitch','yaw'])
	Outcome :
	- success : sucessfully move to the target
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],io_keys=["group","target","frame","offset"])
					
	def acquireFrameUD(self,ud):
		target_list = ast.literal_eval(ud)
		return target_list  
		
	def createOffset(self,target,offset):
		tf_ = tf.TransformerROS()
		target_quat = self.eulerToQuaternion(target)
		offset_quat = self.eulerToQuaternion(offset)
		target_matrix = tf_.fromTranslationRotation([target[0],target[1],target[2]],target_quat)
		offset_matrix = tf_.fromTranslationRotation([offset[0],offset[1],offset[2]],offset_quat)
		new_target_matrix = np.dot(target_matrix, offset_matrix)	
		trans = tf.transformations.translation_from_matrix(new_target_matrix)
		quat = tf.transformations.quaternion_from_matrix(new_target_matrix)
		translation = [trans[0],trans[1],trans[2]]
		quaternion = [quat[0],quat[1],quat[2],quat[3]]
		new_target = translation + quaternion
		return new_target
	
	def eulerToQuaternion(self,angles):
		roll = np.deg2rad(angles[3])
		pitch = np.deg2rad(angles[4])
		yaw = np.deg2rad(angles[5])
		quaternion_tf = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
		quaternion = [quaternion_tf[0],quaternion_tf[1],quaternion_tf[2],quaternion_tf[3]]
		return quaternion
		
	def poseTarget(self,target_pos,frame):
		pose_target = PoseStamped()
		pose_target.header.frame_id = frame
		#Translations 
		pose_target.pose.position.x = target_pos[0]
		pose_target.pose.position.y = target_pos[1]
		pose_target.pose.position.z = target_pos[2]
		#Rotations				
		pose_target.pose.orientation.x = target_pos[3]
		pose_target.pose.orientation.y = target_pos[4]
		pose_target.pose.orientation.z = target_pos[5]
		pose_target.pose.orientation.w = target_pos[6]
		return pose_target
	
	def execution(self,ud):
		print("MOVE CARTESIAN")
		self.group = moveit_commander.MoveGroupCommander(ud.group)
		
		
		#Planification and Execution
		for pos in self.acquireFrameUD(ud.target):
			self.group.clear_pose_targets()
			self.new_pos = self.createOffset(pos,self.acquireFrameUD(ud.offset))
			print(self.new_pos)
			self.group.set_pose_target(self.poseTarget(self.new_pos,ud.frame))
			self.plan = self.group.plan()
			rospy.sleep(2)
			self.group.execute(self.plan)

		return "success"
		


class MoveArti(ssm_state.ssmState):
	'''@SSM
	Description :  Action skill that makes a moveit move group move by commanding its joints (articular) 
	User-data :
	- group : a moveit move group name containing all the joints to move (string)
	- joints : desired angle values of all joints (['J1','J2','J3','J4','J5','J6'])
		      (it could be a list of desired values : [[values1],[values2],[values3]])
	Outcome :
	- success : sucessfully move to all joint targets
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],io_keys=["group","joints"])
	
	def acquireJointsUD(self,ud):
		target_list = ast.literal_eval(ud)
		return target_list  
		
	def execution(self,ud):
		print("MOVE ARTICULAR")
		self.group = moveit_commander.MoveGroupCommander(ud.group)
		
		#Planification and Execution
		for target in self.acquireJointsUD(ud.joints):
			#self.group.clear_pose_targets()
			print(target)
			for i in range (0,6):
				target[i] = np.deg2rad(target[i])
			self.group.set_joint_value_target(target)
			self.plan = self.group.plan()		
			rospy.sleep(2)
			self.group.execute(self.plan)

		return "success"
	
		
		
class Pick(ssm_state.ssmState):
	'''@SSM
	Description :  Action skill that makes the tool pick an object
	User-data :
	- tool : a tool moveit move group name (string)
	- obj : the name of an object to pick (string)
	Outcome :
	- success : sucessfully picked the object
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],io_keys=["tool","obj"])
									  
	def execution(self,ud):
		#Getting the gripper move group
		self.group_gripper = moveit_commander.MoveGroupCommander(ud.tool)
		#Picking execution
		self.group_gripper.attach_object(ud.obj)
		#self.group_gripper.pick(ud.obj)
		rospy.sleep(1) 	
		return "success"



class Place(ssm_state.ssmState):
	'''@SSM
	Description :  Action skill that makes the tool place an object
	User-data :
	- tool : a tool moveit move group name (string)
	- obj : the name of an object to place (string)
	Outcome :
	- success : sucessfully place the object
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],io_keys=["tool","obj"])
									  
	def execution(self,ud):
		#Getting the gripper move group
		self.group_gripper = moveit_commander.MoveGroupCommander(ud.tool)
		#Placing execution
		self.group_gripper.detach_object(ud.obj)
		#self.group_gripper.place(ud.obj)
		rospy.sleep(1)	 
		return "success"



class GenEnvironment(ssm_state.ssmState):
	'''@SSM
	Description :  Skill that generate the environment
	User-data : N/A
	Outcome :
	- success : sucessfully created the environment
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"])
		
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
		self.scene = moveit_commander.PlanningSceneInterface()	
		self.scene.add_box("ground", self.createPose(0,0,-0.01), (10, 10, 0.02))
		self.scene.add_box("table1", self.createPose(0.5,0,0.25), (0.45, 0.45, 0.5))
		self.scene.add_box("table2", self.createPose(0,0.5,0.25), (0.45, 0.45, 0.5))
		rospy.sleep(1)
		return "success"



class InitObject(ssm_state.ssmState):
	'''@SSM
	Description :  Skill that initialize an object to pick
	User-data : N/A
	Outcome :
	- success : sucessfully initialized the object
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"])
			
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
		self.scene = moveit_commander.PlanningSceneInterface()	
		self.scene.add_box("object", self.createPose(0.4,0,0.55), (0.1,0.1,0.1))
		#self.scene.add_box("object", self.createPose(0.4,0.3,0.55), (0.1,0.1,0.1))
		rospy.sleep(1)
		return "success"




"""
class Commander(ssm_state.ssmState):
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success","relaunch"],io_keys=["joints","target"])

	def acquireObjectPos(self,obj):
		obj_string = obj
		obj_list = obj_string.split(" ")
		for i in range (0,6) :
			obj_list[i] = float(obj_list[i])
		
		for i in range (3,6) :
			obj_list[i] = np.deg2rad(obj_list[i])	
				
		return obj_list 
		
	def execution(self,ud):
		
		self.object_name = raw_input("Object name : ")
		self.object_pos_raw = raw_input("Object position : ")
		
		self.object_pos = self.acquireObjectPos(self.object_pos_raw)
		

		print("Object name = %s" %self.object_name)
		print ("Object position = %s" %self.object_pos)
		
		self.joint_1_rot = np.rad2deg(np.arctan(float(self.object_pos[1])/float(self.object_pos[0])))
		print (self.joint_1_rot)
		
		
		ud.joints = "%f 0 0 0 0 0" % np.float(self.joint_1_rot)
		
		print(type(ud.joints))
		rospy.sleep(20)

		self.step = int(ud.step)
			
		if (self.step == 0):
			ud.joints = "%f 0 0 0 0 0" % np.float(self.joint_1_rot)
			rospy.sleep(20)
			return "relaunch"
		
		elif (self.step == 1):
			ud.joints = "%f 20 20 0 0 0" % np.float(self.joint_1_rot)
			return "relaunch"
			
		elif (self.step > 1):
			return "success"
	
		
		else :
			print("YOUR CODE IS NOT WORKING LIKE YOU WOULD LIKE")
			return "success"

		return "relaunch"
		"""


	  
class Sensor(ssm_state.ssmState):
	'''@SSM
	Description :  Skill that wait for the sensor information (position of an object)
	User-data :
	- objpos : the position of the object
	Outcome :
	- success : sucessfully got the information
	'''
	def __init__(self):
		ssm_state.ssmState.__init__(self,outcomes=["success"],io_keys=["objpos"])
		self.pub = rospy.Publisher('chatter', PoseStamped, queue_size=10)
		self.msg = PoseStamped()
		self.msg.pose.position.x=0
		self.msg.pose.position.y=0
		self.msg.pose.position.z=0
		
		
	def callback(self,data):
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", (data.pose.position.x,data.pose.position.y,data.pose.position.z))	
		self.x=data.pose.position.x
		self.y=data.pose.position.y
		self.z=data.pose.position.z
		
	def execution(self,ud):	
						
		self.sensor_info = rospy.Subscriber("/chatter", PoseStamped, self.callback)		
		rospy.wait_for_message("/chatter", PoseStamped)
		
		print("RECU")
		print(self.x)
		print(self.y)
		print(self.z)
		
		ud.objpos = [self.x,self.y,self.z]
		
		return "success"



