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

import os
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix
from cobot_gui import Plugin, ControlMode

## Finally import the RViz bindings themselves.
import rviz


## The MyViz class is the main container widget.
class PluginRviz(Plugin):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self, context):
        Plugin.__init__(self, context)
        
        self.frame = None
        
    def onCreate(self, param):
        
        yaml = param.getParam("yaml")
        yaml = get_pkg_dir_from_prefix(yaml)
        
        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels. In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()
        
        ## The "splash path" is the full path of an image file which
        ## gets shown during loading. Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath("")
        
        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load(). In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()
        
        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        
        reader.readFile( config, yaml)
        self.frame.load( config )
        
        ## You can also store any other application data you like in
        ## the config object. Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        
        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setHideButtonVisibility( False )
        
        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class. It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()
        
        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control. Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)
        
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        
        #######
        
        self.setLayout( layout )
    
    def onPause(self):
        pass
    
    def onResume(self):
        pass
    
    def onControlModeChanged(self, mode):
        
        if mode == ControlMode.AUTOMATIC:
            self.setEnabled(False)
        else:
            self.setEnabled(True)
    
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass

