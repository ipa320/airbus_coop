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

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber
from cobot_gui import dashboard

from template_dashboard.res import R

class TemplatePopup(dashboard.DashboardPopup):
    
    def __init__(self, parent):
        """ Popup constructor.
        It would be better not to add code here.
        You can change the popup instance positions (relative position beetwin popup/dashboarditem):
         - TopLeft,
         - BottomRight,
         - TopRight,
         - BottomLeft,
         - Center.
        """
        dashboard.DashboardPopup.__init__(self,
                                          parent,
                                          dashboard.DashboardPopup.TopRight,
                                          dashboard.DashboardPopup.BottomRight)
        
    def onCreate(self, param):
        """ Load and sets your GUI components
        """
        # Load the ui file
        loadUi(R.layouts.popup, self)
        
        # Set default style sheet
        self.title_label.setStyleSheet(R.values.styles.hello)
        
    def onTranslate(self, lng):
        """ This method is called when the user change the language.
        You can retanslate your popup here.
        
        Example: 
        
        self.popup_label.setText(R.values.strings.$MY_STRING_ID(lng))
        
        When $MY_STRING_ID defite into strings.xml (<string id="$MY_STRING_ID">...</string>)
        
        """
        self.title_label.setText(R.values.strings.hello(lng))
    
    def onDestroy(self):
        """ This method is called when popup closes.
        You can delete memory and connections
        """

class TemplateDashboard(dashboard.Dashboard):
    
    def __init__(self, context):
        """ Dashboard constructor.
        It would be better not to change anything here.
        """
        dashboard.Dashboard.__init__(self, context)
        
    def onCreate(self, param):
        """ Load and sets your GUI components
        @param param: C{Parameters}
        
        onCreate set the parameters from the dashboard_descriptor.xml (params element) from the cobot_gui config ($MY_COBOT_GUI.conf)
        
        Example:
        my_param = param.getParam($MY_PARAM_NAME, $MY_DEFAULT_VALUE)
        """
        
        # Example:
        #  - Create a label to display a status image
        self._status_label = QLabel()
        #  - Putting and resizing the image in the label
        self._status_label.setPixmap(R.getPixmapById("ic_dashboard").scaled(
                                     self.width(),
                                     self.height(),
                                     Qt.KeepAspectRatio,
                                     Qt.SmoothTransformation))
        
        #  - Add label to dashboard layout
        self.getLayout().addWidget(self._status_label)
        
    def onRequestPopup(self):
        """ This method is called when the user clicks on the dashboard label/icon.
        If you don't want to popup you should return None.
        Else you should return the popup instance.
        """
        return TemplatePopup(self)
        
    def onControlModeChanged(self, mode):
        """ This method is called when the user change the control mode.
        @param mode: C{ControlMode}
        
        You can write rules when the control mode changes.
        
        Example:
        NB: import needed (from cobot_gui import ControlMode)
        
        if mode == ControlMode.AUTOMATIC:
            # Do somethings
        elif mode == ControlMode.MANUAL:
            # Do somethings
        """
    
    def onTranslate(self, lng):
        """ This method is called when the user change the language.
        You can retanslate your dashboard here.
        
        Example: 
        
        self.dashboard_label.setText(R.values.strings.$MY_STRING_ID(lng))
        
        When $MY_STRING_ID defite into strings.xml (<string id="$MY_STRING_ID">...</string>)
        
        """
    
    def onEmergencyStop(self, state):
        """ This method is called when the emergency stop state changed.
        You can make emergency action here.
        
        Exemple:
        
        if state == EmergencyStopState.LOCKED:
            # Do somethings
        elif state == EmergencyStopState.UNLOCKED:
            # Do somethings
        """
    
    def onDestroy(self):
        """ This method is called when cobot_gui closes.
        You can free memory and disconnects topics
        """
    
if __name__ == "__main__":
    
    import sys
    from cobot_gui import dashboard
     
    rospy.init_node("template_dashboard_node")
     
    a = QApplication(sys.argv)
     
    window = dashboard.getStandAloneInstance("template_dashboard", TemplateDashboard, "en")
    window.setWindowTitle("TemplateDashboard")
    window.show()
    a.exec_()
