#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : cobot_gui_node.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

## @package: cobot_gui_node
##
## @version 2.0
## @author  Matignon Martin
## @date    Last modified 21/02/2014
import rospy
from roslib.packages import get_pkg_dir
import sys
import os
import subprocess
from xml.etree import ElementTree

from pyqt_agi_extend import QtAgiCore

# Load my resources file
from cobot_gui.res import R

def set_boot_configuration(config):
    
    autorun = ElementTree.Element('boot')
    autorun.set('config', config)
    tree = ElementTree.ElementTree(autorun)
    
    tree.write(R.DIR+'/autorun.xml', encoding='utf8', method='xml')

if __name__ == "__main__":
    
    rospy.init_node('cobot_gui_autorun_%d' % os.getpid())
    
    config = rospy.get_param("~config", "${cobot_gui}/resources/config/default.conf")
    
    set_boot_configuration(config)
    
    subprocess.Popen(['rosrun', 'cobot_gui','autorun.py'])
    
#@endcond

