#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : autorun.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

## @package: autorun
##
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 21/02/2014

import rospy
import sys
import os
import subprocess
import time

from xml.etree import ElementTree as ET

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from roslib.packages import get_pkg_dir

# Load my generated resources file
from cobot_gui.res import R

class CommandShell:
    
    def __init__(self):
        self._cmds=[]
        self._proc = None
    
    def __str__(self):
        return ('\n'.join(self._cmds)).encode('utf-8')
    
    def compile(self):
        return self.__str__()
    
    def putCmd(self, cmd, *argv):
        compile_args = ''
        for arg in argv:
            if isinstance(arg, list):
                for subarg in arg:
                    compile_args+='%s '%subarg
            else:
                compile_args+='%s '%arg
        
        self._cmds.append('%s %s'%(cmd, compile_args))
    
    def call(self):
        
        self._proc = subprocess.Popen('/bin/bash', stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        out, err = self._proc.communicate(self.__str__())
    
    def kill(self):
        if self._proc is not None:
            self._proc.kill()
    
class RosCoreThread(QThread):
    
    def __init__(self, parent):
        QThread.__init__(self, parent)
        
        self._keep_running = True
        self._shell = CommandShell()
        
    def shutdown(self):
        self._keep_running = False
        
    def spin(self):
        
        while self._keep_running:
            self.msleep(500)
        
    def run(self):
        #self._shell.putCmd("source", "/opt/ros/indigo/workspace/devel/setup.bash")
        #self._shell.putCmd("export", "ROS_WORKSPACE=/opt/ros/indigo/workspace")
        #self._shell.putCmd("export", "ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH")
        self._shell.putCmd("roscore")
        
        self._shell.call()
        
        self.spin()
        
        self._shell.kill()
        
class AgiLauncher(QThread):
    
    LOCALHOST = 0
    EXTERNAL  = 1
    
    def __init__(self, parent):
        QThread.__init__(self, parent)
        self._target = self.LOCALHOST
        self._host = ""
        self._ip   = ""
        self._uri  = ""
        self._launch = ""
        self._shell = CommandShell()
        
    def setTarget(self, target):
        self._target = target
        
    def setRosHostname(self, host):
        self._host = host
        
    def setRosIp(self, ip):
        self._ip = ip
        
    def setRosMasterUri(self, uri):
        self._uri = uri
        
    def setLaunchCmd(self, cmd):
        self._launch = cmd
        
    def shutdown(self):
        self._shell.kill()
        
    def spin(self):
        
        while self._keep_running:
            self.msleep(100)
        
    def run(self):
        
        #self._shell.putCmd("source", "/opt/ros/indigo/workspace/devel/setup.bash")
        #self._shell.putCmd("export", "ROS_WORKSPACE=/opt/ros/indigo/workspace")
        #self._shell.putCmd("export", "ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH")
        
        if self._target == self.EXTERNAL:
            self._shell.putCmd("export", "ROS_HOSTNAME=%s"%self._host)
            self._shell.putCmd("export", "ROS_IP=%s"%self._ip)
            self._shell.putCmd("export", "ROS_MASTER_URI=%s"%self._uri)
            
        self._shell.putCmd("roslaunch",self._launch)
        
        print str(self._shell)
        
        self._shell.call()

class MenuLauncher(QWidget):
    
    def __init__(self):
        QWidget.__init__(self)
        
        loadUi(R.layouts.menu_launcher2, self)
        
        self.connect(self.launcher_list, SIGNAL('currentIndexChanged(int)'), self.launcherSelectionChanged)
        self.connect(self.start_button, SIGNAL('clicked()'), self.startLaunch)
        
        self.setup()
        
    def setup(self):
        
        self._launchers_root = ET.parse(R.values.launchers)
        self._launchers_tree = self._launchers_root.getroot()
        
        for launcher in self._launchers_tree.iter("launcher"):
            self.launcher_list.addItem(launcher.attrib['name'])
            
        self._roscore = RosCoreThread(self)
        self._process_launch = AgiLauncher(self)
        
    def launcherSelectionChanged(self, index):
        
        launch_name = self.launcher_list.itemText(index)
        launcher_node = self._launchers_tree.find('launcher[@name="%s"]'%launch_name)
        ros_config_name = launcher_node.attrib["ros-config"]
        ros_config_node = self._launchers_tree.find('ros-configs/roscore[@name="%s"]'%ros_config_name)
        
        self.launch_file_info.setText(launcher_node.attrib["launch"])
        self.ros_hostname_info.setText(ros_config_node.find("ROS_HOSTNAME").text)
        self.ros_ip_info.setText(ros_config_node.find("ROS_IP").text)
        self.ros_master_uri_info.setText(ros_config_node.find("ROS_MASTER_URI").text)
        
    def startLaunch(self):
        
        self._process_launch.shutdown()
        self._roscore.shutdown()
        
        if self.ros_hostname_info.text() != "127.0.0.1":
            self._process_launch.setTarget(AgiLauncher.EXTERNAL)
            self._process_launch.setRosHostname(self.ros_hostname_info.text())
            self._process_launch.setRosIp(self.ros_ip_info.text())
            self._process_launch.setRosMasterUri(self.ros_master_uri_info.text())
        else:
            self._roscore.start()
            time.sleep(1)
            
        self._process_launch.setLaunchCmd(self.launch_file_info.text())
        
        self._process_launch.start()
        
    def closeEvent(self, event):
        self._roscore.shutdown()
        
if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    
    window = QMainWindow()
    window.setWindowTitle("Cobot_gui configuration")
    
    window.setCentralWidget(MenuLauncher())
    
    window.show()
    
    app.exec_()
    
#@endcond

