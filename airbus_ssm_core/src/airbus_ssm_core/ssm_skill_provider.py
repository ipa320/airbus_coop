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

import __builtin__
import os
import sys

import rospy

from roslib.packages import get_pkg_dir
from xml.etree import ElementTree
from importlib import import_module

class SkillProvider(object):
    
    def __init__(self, skill_register_file):
        
        if not os.path.isfile(skill_register_file):
            raise Exception("Invalid register file path !")
        
        self._tree = ElementTree.parse(skill_register_file)
        self._root = self._tree.getroot()
    
    def load(self, name):
        
        """ Find skill registered by name
        <skill name="${name}" pkg="${pkg}" module="${module}" class="${class}"/>
        """
        
        skill = self._root.find('skill[@name="%s"]'%name)
        if skill is not None:
            try:
                in_pkg          = skill.attrib["pkg"]
                in_module       = skill.attrib["module"]
                skill_class     = skill.attrib["class"]
                skill_class_ref = None
                skill_class_ref = import_module(in_module, in_pkg).__getattribute__(skill_class)
                
                return skill_class_ref
            except Exception as ex:
                return None
        else:
            rospy.logerr('[SCXML Interpreter] Skill named "%s" not found in register file !'%name)
            return None
    
    
    
