#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : env.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import os
import rospy
from roslib.packages import get_pkg_dir
import catkin_pkg.workspaces

OUTPUT = rospy.get_param('/agi_docgen/output_path','/tmp/docu')
ROSDOC_ROOT      = get_pkg_dir("agi_docgen")# os.path.join(ROS_WS, "rosdoc")
ROSDOC_RSC       = os.path.join(OUTPUT)
ROSDOC_DOT       = os.path.join(ROSDOC_RSC, "dot")
ROSDOC_IMAGES    = os.path.join(ROSDOC_RSC, "images")
ROSDOC_POLICES   = os.path.join(ROSDOC_RSC, "polices")
ROSDOC_STYLES    = os.path.join(ROSDOC_RSC, "styles")
ROSDOC_TEMPLATES = os.path.join(ROSDOC_RSC, "templates")
ROSDOC_GEN = os.path.join(OUTPUT, "gen")
