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

import rospy
import os
import datetime
import sys
from airbus_ssm_core import ssm_skill_provider
from airbus_ssm_core.ssm_scxml_interpreter import get_pkg_dir_from_prefix
from importlib import import_module
from xml.etree import ElementTree

def StrTimeStamped():
    return datetime.datetime.now().strftime("%Y_%m_%d_%H:%M")


class SkillXMLDescriptor():
    
    def __init__(self, input_file, output_file):
        #Check the existence of the skill xml file
        self._input_file = get_pkg_dir_from_prefix(input_file)
        if(os.path.isfile(self._input_file) == False):
            self._input_file = get_pkg_dir_from_prefix("${airbus_ssm_core}/resources/"+input_file)
            if(os.path.isfile(self._input_file) == False):
                rospy.logerr(["[SkillXMLDescriptor] '%s' not found !"%input_file])
                sys.exit(-1)
        #Check the output file
        self._output_file = get_pkg_dir_from_prefix(output_file)
        try:
            f = open(self._output_file, 'w')
            f.close()
        except Exception as e:
            rospy.logerr("[SkillXMLDescriptor] error during output file creation : %s"%e)
            sys.exit(-1) 

        #Create the SkillProvider
        try:
            self._SkillProvider = ssm_skill_provider.SkillProvider(self._input_file)
        except Exception as e:
            rospy.logerr("[SkillXMLDescriptor] error during skill provider creation : %s"%e)
            sys.exit(-1)
        
        self.describeXMLFile()
        
    
    def describeXMLFile(self):
        
        _tree = ElementTree.parse(self._input_file)
        _root = _tree.getroot()
        _descriptors = []
        _descriptors.append("Skill XML File Descriptor")
        _descriptors.append("XML File : '%s'"%self._input_file)
        _descriptors.append("Creation : '%s'"%StrTimeStamped())
        _descriptors.append('')
        _descriptors.append("-----------------------------------------------------------------")
        for skill in _root:
            _state = self._SkillProvider.load(skill.attrib["name"])()
            _path = (import_module(skill.attrib["module"], skill.attrib["pkg"]).__file__)
            _path = _path[:-1]
            _descr = self.find_descriptions(_path, skill.attrib["class"])
            _outcomes = self.find_outcomes(_state)
            _iokeys = self.find_iokey(_state)
            _descriptor = self.createDescriptor(skill.attrib["name"], skill.attrib["class"], _descr, _outcomes, _iokeys)
            for line in _descriptor:
                _descriptors.append(line)
            
        self.writeTxtFile(_descriptors)
        
    
    def createDescriptor(self, name, class_, decription, outcomes, io_keys):
        descriptor = []
        descriptor.append("State Name : '%s' (used in the scxml file)"%name)
        descriptor.append("Class      : '%s'"%(class_))
        descriptor.append(outcomes)
        descriptor.append(io_keys)
        descriptor.append('')
        for line in decription:
            descriptor.append(line)
        descriptor.append("-----------------------------------------------------------------")
        return descriptor
        
            
    def writeTxtFile(self, descriptors):
        file = open(self._output_file, 'w')
        for line in descriptors:
            file.write("%s\n"%line)
        file.close()
        
        rospy.loginfo("[SkillXMLDescriptor] XML file : '%s'"%self._input_file)
        rospy.loginfo("[SkillXMLDescriptor] Description file created : '%s'"%self._output_file)
        
    
    def find_outcomes(self, state):
        _str = "Outcomes   : "
        _outcomes = state.get_registered_outcomes()
        for outcome in _outcomes:
            if(outcome != "preempt"):
                _str = _str +"'"+outcome + "', "
        _str = _str[:-2]
        
        return _str
        
    def find_iokey(self, state):
        _str = "User data  : "
        _keys = state.get_registered_input_keys()
        if(len(_keys) > 2):
            for key in _keys:
                 if(key != "skill" and key != "logfile"):
                     _str = _str +"'"+key + "', "
            _str = _str[:-2]
        else:
            _str = "No user data."
        
        return _str
        
    def find_descriptions(self, file, state_py):
        _file = open(file).readlines()
        _search_str = "class "+state_py
        _file = [x.strip('\n') for x in _file]
        _found = 0
        #Find the class we are working on
        for num, line in enumerate(_file,1):
            if _search_str in line:
                _found = num
                break
        _file = _file[_found:]
        #Limit the reseach on this class (find the next one if it exist and keep only what is before that)
        _found = -1
        _search_str = "class "
        for num, line in enumerate(_file[2:],1):
            if _search_str in line:
                _found = num
                break
        if(_found != -1):
            _file = _file[:_found]
        #Look for a decription tag
        _found = -1
        _search_str = "'''@SSM"
        for num, line in enumerate(_file,1):
            if _search_str in line:
                _found = num
                break
        if(_found != -1):
            _file = _file[_found:]
            #Look for the end decription tag
            _search_str = "'''"
            for num, line in enumerate(_file[1:],1):
                if _search_str in line:
                    _found = num
            _str = []
            for line in _file[:_found]:
                _str.append(line.strip())
        else:
            _str = ["No description found for this class.","Check the python file.", str(file)]
            rospy.logwarn("[SkillDescriptor] : No description found for the class : '%s'"%state_py)
        
        return _str


if __name__ == '__main__':
    rospy.init_node("ssm_descriptor")
    input_file = rospy.get_param('/ssm_descriptor/skill_xml_file', default='empty_register.xml')
    output_file = rospy.get_param('/ssm_descriptor/output_file', default='/tmp/descriptor.txt')
    descriptor = SkillXMLDescriptor(input_file, output_file)
    
