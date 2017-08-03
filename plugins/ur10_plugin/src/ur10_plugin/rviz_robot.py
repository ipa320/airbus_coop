#!/usr/bin/env python

import rospy
import os

from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi
from ur10_plugin.res import R

import rviz

## The MyViz class is the main container widget.
class RVizRobot:
    
    def __init__(self, parent):
        
        self._parent = parent
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, R.yamls.robot_arm)
        self.frame.load( config )
        self.widget = self.frame.centralWidget()
        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setHideButtonVisibility( False )
        
        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()
        
        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
    def get_widget(self):
        return self.widget
        
    def resize(self, size):
        self.widget.setFixedSize(QSize(size.width() -2, size.height() -2))
        
#End of file

