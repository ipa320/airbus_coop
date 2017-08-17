#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
# Copyright 2015 Airbus
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
