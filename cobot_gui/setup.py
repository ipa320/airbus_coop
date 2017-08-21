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
