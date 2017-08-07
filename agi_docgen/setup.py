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
	packages=['agi_docgen',
            'agi_docgen.common',
            'agi_docgen.digraph',
            'agi_docgen.digraph.model',
            'agi_docgen.docgen',
            'agi_docgen.docgen.config',
            'agi_docgen.docgen.gui',
            'agi_docgen.docgen.home',
            'agi_docgen.docgen.pkg',
            'agi_docgen.docgen.pkg.node'],
	package_dir={'': 'src'},
)

setup(**d)
