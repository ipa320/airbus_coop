#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : setup.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	packages=['cobot_gui',
	'cobot_gui.account',
	'cobot_gui.alarm',
	'cobot_gui.util',
	'cobot_gui.control_mode',
	'cobot_gui.dashboard',
	'cobot_gui.emergency',
	'cobot_gui.plugin',
	'cobot_gui.timestamp',
	'cobot_gui.translator'],
	package_dir={'': 'src'},
)

setup(**d)
