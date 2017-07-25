#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : iiwa_interface_node.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy
import sys

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from iiwa_plugin.plugin import IIWAPlugin

from cobot_gui import plugin

if __name__ == "__main__":
    
    import sys
    
    rospy.init_node("iiwa_interface_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("iiwa_plugin", IIWAPlugin)
    window.setWindowTitle("IIWA Interface")
    window.show()
    a.exec_()
    
#End of file

