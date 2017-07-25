#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin_installer.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from roslib.packages import get_pkg_dir
from convert_tools import *
from xml.etree import ElementTree
from xml.etree.ElementTree import tostring
from xml.dom import minidom

# import pyaudio
import wave

from cobot_gui import plugin, Alarm
from trajectory_xml.res import R

chunk = 1024

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = tostring(elem, 'utf-8', method="xml")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="\t")



## @package: plugin
##
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 28/02/2014

## @class Plugin
## @brief Plugin interface..
class TrajectoryXmlPlugin(plugin.Plugin):
    
    def __init__(self, context):
        plugin.Plugin.__init__(self, context)
        
    def onCreate(self, param):
        
        loadUi(R.layouts.mainwindow, self)
        
        self.AutoDetect = False
        self.GravityOn = False
        self.Articular_sub = None
        self.Cartesian_sub = None
        self.RsiMode_pub = None
        self.Cartesian = []
        self.Articular = []
        self.MaxArticular = 1.0
        self.MaxCart = 1.0
        self.Articular_act = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.Cartesian_act = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.Articular_lastpose = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.Cartesian_lastpose = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.TorqueList = []
        self.GetTorque = True
        self.SavedTorque = 0.0
        self.ActualTorque = 0.0
        self.NbPt = 1
        self.JointPt = 1
        self.ArticularList = []
        self.CartesianList = []
        self.root = ElementTree.Element("root")
        tag = str(self.PointName.toPlainText())
        self.PointElmt = ElementTree.SubElement(self.root, tag)
        
        self.connect(self.GravitySwitch_Button,
                     SIGNAL("clicked()"),
                     self.SwitchGravity)
        
        self.connect(self.AddPtArticular,
                     SIGNAL("clicked()"),
                     self.AddPtArticular_line)
        
        self.connect(self.AddPtCartesian,
                     SIGNAL("clicked()"),
                     self.AddPtCartesian_line)
        
        self.connect(self.Save_Button,
                     SIGNAL("clicked()"),
                     self._slot_Save)
        
        self.connect(self.NewTraj,
                     SIGNAL("clicked()"),
                     self._slot_newTraj)
        
        self.Timer_Refresh = rospy.Timer(rospy.Duration(0.3), self.Refresh_cb)
        
    
    def onPause(self):
        if self.Articular_sub is not None:
            self.Articular_sub.unregister()
            self.Cartesian_sub.unregister()
            self.RsiMode_pub.unregister()
    
    def onResume(self):
        self.Articular_sub = rospy.Subscriber('/lwr/joint_states', JointState, self.Articular_cb)
        self.Cartesian_sub = rospy.Subscriber('/lwr/pose_cart', PoseStamped, self.Cartesian_cb)
        self.RsiMode_pub = rospy.Publisher('/lwr/prog_cmd', Int8, queue_size=5)
        
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        self.Name.setText(R.values.strings.title(lng))
    
    def onEmergencyStop(self, state):
        self.onPause()
    
    def Articular_cb(self, data):
        self.Articular_act = [data.position[0]*180/math.pi,data.position[1]*180/math.pi,data.position[2]*180/math.pi,data.position[3]*180/math.pi,
                              data.position[4]*180/math.pi,data.position[5]*180/math.pi,data.position[6]*180/math.pi]
        if(self.GetTorque):
            self.TorqueList.append(data.effort[3])
            if(len(self.TorqueList) >= 30):
                sum = 0.0
                for i in range(len(self.TorqueList)):
                    sum += self.TorqueList[i]
                
                self.SavedTorque = sum/len(self.TorqueList)
                self.TorqueList = []
                self.GetTorque=False
                
        if(self.AutoDetect == True and self.GravityOn == False and self.GetTorque == False):
            self.ActualTorque = data.effort[3]
            self.AutoDetection()
    
    def Cartesian_cb(self, data):
        self.Cartesian_act = FormatPose("deg",Format44quat([data.pose.position.x,data.pose.position.y,data.pose.position.z,
                              data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]))
                        
    def Refresh_cb(self,event):
        self.A1Act.setText(str("A1 = %.4f" % self.Articular_act[0]))
        self.A2Act.setText("A2 = %.4f" % self.Articular_act[1])
        self.A3Act.setText("A3 = %.4f" % self.Articular_act[2])
        self.A4Act.setText("A4 = %.4f" % self.Articular_act[3])
        self.A5Act.setText("A5 = %.4f" % self.Articular_act[4])
        self.A6Act.setText("A6 = %.4f" % self.Articular_act[5])
        self.A7Act.setText("A7 = %.4f" % self.Articular_act[6])
        
        self.XAct.setText("X = %.4f" % self.Cartesian_act[0])
        self.YAct.setText("Y = %.4f" % self.Cartesian_act[1])
        self.ZAct.setText("Z = %.4f" % self.Cartesian_act[2])
        self.AAct.setText("A = %.4f" % self.Cartesian_act[3])
        self.BAct.setText("B = %.4f" % self.Cartesian_act[4])
        self.CAct.setText("C = %.4f" % self.Cartesian_act[5])
                    
    def AddPtArticular_line(self):
        self.AddLine(False,True)
    
    def AddPtCartesian_line(self):
        self.AddLine(True,False)
    
    def AddLine(self, cart = True, articular = True):
#         self.PlaySound()
        self.Articular = []
        self.Cartesian = []
        self.Articular_lastpose = self.Articular_act
        self.Cartesian_lastpose = self.Cartesian_act
        if(cart):
            if(self.JointPt>1):
                self._slot_newTraj()
            else:
                tag = str(self.PointName.toPlainText())
                self.PointElmt = ElementTree.SubElement(self.root, tag, type = "cart")
            ElementTree.SubElement(self.PointElmt, "position", 
                                              x="%f"%self.Cartesian_act[0],
                                              y="%f"%self.Cartesian_act[1],
                                              z="%f"%self.Cartesian_act[2])
            
            ElementTree.SubElement(self.PointElmt, "orientation",
                                              type = "degEulerZYX", 
                                              a="%f"%self.Cartesian_act[3],
                                              b="%f"%self.Cartesian_act[4],
                                              c="%f"%self.Cartesian_act[5])
            self._slot_newTraj()
        
        if(articular):
            ElementTree.SubElement(self.PointElmt, ("joint"+str(self.JointPt)),
                                              type="deg", 
                                              a1="%f"%self.Articular_act[0],
                                              a2="%f"%self.Articular_act[1],
                                              a3="%f"%self.Articular_act[2],
                                              a4="%f"%self.Articular_act[3],
                                              a5="%f"%self.Articular_act[4],
                                              a6="%f"%self.Articular_act[5],
                                              a7="%f"%self.Articular_act[6])
            self.JointPt += 1 
    
    def AutoDetection(self):
        delta_torque = self.ActualTorque - self.SavedTorque
        if(delta_torque < -1 or delta_torque > 1):
            self.TurnOnGravity()
    
    def SwitchGravity(self):
        if(self.GravityOn == False):
            self.GravitySwitch_txt.setText("Gravity : ON")
            self.RsiMode_pub.publish(50)
            rospy.sleep(2)
            self.RsiMode_pub.publish(2)
            self.GravityOn = True
        else:
            self.GravitySwitch_txt.setText("Gravity : OFF")
            self.RsiMode_pub.publish(51)
            rospy.sleep(2)
            self.RsiMode_pub.publish(2)
            self.GravityOn = False
            
    def _slot_newTraj(self):
        self.JointPt=1
        self.NbPt += 1
        self.PointName.setPlainText("Point%d" % self.NbPt)
        tag = str(self.PointName.toPlainText())
        self.PointElmt = ElementTree.SubElement(self.root, tag, type = "joint")     
            
    def _slot_Save(self):
        element = prettify(self.root)
        directory = get_pkg_dir("trajectory_xml")+"/save/"
        file  = open(directory+self.TrajectoryName.toPlainText() + ".xml","w")
        file.write(element)
        file.close()
        
#     def PlaySound(self):
#         #open a wav format music 
#         directory = get_pkg_dir("trajectory_xml")+"/resources/"
#         f = wave.open(directory + "beep-08b.wav" ,"rb")  
#         #instantiate PyAudio  
#         p = pyaudio.PyAudio()  
#         #open stream  
#         stream = p.open(format = p.get_format_from_width(f.getsampwidth()),  
#                         channels = f.getnchannels(),  
#                         rate = f.getframerate(),  
#                         output = True)  
#         #read data  
#         data = f.readframes(chunk)  
#         
#         #paly stream  
#         while data != '':  
#             stream.write(data)  
#             data = f.readframes(chunk)  
#         
#         #stop stream  
#         stream.stop_stream()  
#         stream.close()
    
    def onDestroy(self):
        self.onPause()
    
    
#End of file

