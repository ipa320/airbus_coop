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
from roslib.packages import get_pkg_dir
from xml.etree import ElementTree

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix

from cobot_gui.cobot_gui_main import CobotGuiSplash, \
                                     CobotGuiMain
                                     
from pyqt_agi_extend import QtAgiCore

# Load my generated resources file
from cobot_gui.res import R

def get_boot_configuration():
    
    file_dir = R.DIR+'/autorun.xml'
    
    autorun = ElementTree.parse(file_dir)
    config = autorun.getroot()
    config = config.attrib['config']
    config = get_pkg_dir_from_prefix(config)
    
    os.remove(file_dir)
    
    return config

FULL_SCREEN_ARGS = ["full-screen"   ,"full"   ,"f",
                    "-full-screen"  ,"-full"  ,"-f",
                    "--full-screen" ,"--full" ,"--f"]

if __name__ == "__main__":
    
    name = 'rqt_gui_py_node_%d' % os.getpid()
    rospy.init_node(name, disable_signals=True)
    
    app = QApplication(sys.argv)
    
    splash = CobotGuiSplash()
    splash.start()
    
    window = QMainWindow()
    
    gui = CobotGuiMain(splash)
    gui.setupUserConfig(get_boot_configuration())
    
    window.setCentralWidget(gui)
    window.setGeometry(gui.geometry())
    window.setWindowIcon(R.getIconById('cobot_gui'))
    
    if gui.getDisplayMode() in FULL_SCREEN_ARGS:
        window.showFullScreen()
    else:
        window.show()
    
    splash.close()
    
    app.exec_()
    
    gui.shutdown()
    
#@endcond

