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
from roslib.packages import get_pkg_dir

ROS_WS = os.getenv("ROS_WORKSPACE")
ROSDOC_ROOT      = get_pkg_dir("agi_docgen")# os.path.join(ROS_WS, "rosdoc")
ROSDOC_RSC       = os.path.join(ROSDOC_ROOT, "resources")
ROSDOC_DOT       = os.path.join(ROSDOC_RSC, "dot")
ROSDOC_IMAGES    = os.path.join(ROSDOC_RSC, "images")
ROSDOC_POLICES   = os.path.join(ROSDOC_RSC, "polices")
ROSDOC_STYLES    = os.path.join(ROSDOC_RSC, "styles")
ROSDOC_TEMPLATES = os.path.join(ROSDOC_RSC, "templates")
ROSDOC_GEN = os.path.join(ROSDOC_RSC, "gen")

