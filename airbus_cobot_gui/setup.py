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
	packages=['airbus_cobot_gui',
	'airbus_cobot_gui.account',
	'airbus_cobot_gui.alarm',
	'airbus_cobot_gui.control_mode',
	'airbus_cobot_gui.dashboard',
	'airbus_cobot_gui.diagnostics',
	'airbus_cobot_gui.emergency',
	'airbus_cobot_gui.plugin',
	'airbus_cobot_gui.timestamp',
	'airbus_cobot_gui.translator',
	'airbus_cobot_gui.util'],
	package_dir={'': 'src'},
)

setup(**d)
