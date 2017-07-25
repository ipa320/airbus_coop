#!/usr/bin/env python

import rospy
import os

from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi
from iiwa_plugin.res import R

import rviz

## The MyViz class is the main container widget.
class RobotUi:#KukaLwrWidget:
    
#     RSC_DIR  = get_pkg_dir('plugin_robot_arm_control')+"/resources"
#     YAML_DIR = RSC_DIR+"/yaml"
    VIEW_1 = 'view_1'
    VIEW_2 = 'view_2'
    VIEW_3 = 'view_3'
    VIEW_4 = 'view_4'
    
    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
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
        
        self.setup_ui()
        
    def setup_ui(self):
        
        for i in range(1,5):
            button = getattr(self._parent, "robot_view_%i_button"%i)
            button.setIcon(R.getIconById("robot_view_%i"%i))
            button.setIconSize(QSize(40,40))
            QObject.connect(button, SIGNAL('clicked()'),
                            getattr(self,"_slot_robot_view_%i"%i))
            
            self._slot_robot_view_1()
            
    def _slot_robot_view_1(self):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == self.VIEW_1:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
    
    def _slot_robot_view_2(self):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == self.VIEW_2:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
    
    def _slot_robot_view_3(self):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == self.VIEW_3:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
    
    def _slot_robot_view_4(self):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == self.VIEW_4:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        
    def get_widget(self):
        return self.widget
        
    def resize(self, size):
        self.widget.setFixedSize(QSize(size.width() -2, size.height() -2))
        
#End of file

