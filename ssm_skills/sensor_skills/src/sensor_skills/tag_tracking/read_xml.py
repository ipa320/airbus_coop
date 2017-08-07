#!/usr/bin/env python

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : robot_skills.py
# Authors : Clement Beaudoing
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Clement Beaudoing <clement.beaudoing.external@airbus.com>
#
#
################################################################################


import rospy
from roslib.packages import get_pkg_dir
from xml.etree import ElementTree
from geometry_msgs.msg  import PoseStamped
from convertion_tools import *
import tf
import numpy

class Point:
    def __init__(self, positions=[]):
        self.positions = positions

def ReadXmlPoint(xml_pt):
        
    xpose = xml_pt.find('position')
    xori  = xml_pt.find('orientation')
    
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = float(xpose.attrib['x'])
    pose.pose.position.y = float(xpose.attrib['y'])
    pose.pose.position.z = float(xpose.attrib['z'])
    if (str(xori.attrib['type']) == "degEulerZYX"):
        quat = EulerZYXToQuat(type = "deg", A = float(xori.attrib['a']), B = float(xori.attrib['b']),  C = float(xori.attrib['c']))
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
    if (str(xori.attrib['type']) == "quat"):
        pose.pose.orientation.x = float(xori.attrib['x'])
        pose.pose.orientation.y = float(xori.attrib['y'])
        pose.pose.orientation.z = float(xori.attrib['z'])
        pose.pose.orientation.w = float(xori.attrib['w'])
    return pose


def GetPoseFromXml(goal ='home',file_name = 'goals.xml'):
    
    xml = ElementTree.parse(get_pkg_dir('demo') +'/ressources/' + file_name)
    root = xml.getroot()

    goal_xml = root.find(goal)

    if goal_xml is None:
        return None
    
    pose = ReadXmlPoint(goal_xml)

    return pose

def GetMatrixFromXml(goal ='home',file_name = 'goals.xml', unit = "mm"):
    convert = 1.0
    if(unit == "m"):
        convert = 1000.0
    pose = GetPoseFromXml(goal,file_name)
    translation = tf.transformations.translation_matrix((pose.pose.position.x*convert,pose.pose.position.y*convert,pose.pose.position.z*convert)) 
    rotation   = tf.transformations.quaternion_matrix((pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))   
    matrix = numpy.dot(translation, rotation)
    return matrix
    
