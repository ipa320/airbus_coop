#/usr/bin/env python

import __builtin__
import os
import sys

import rospy

from roslib.packages import get_pkg_dir
from xml.etree import ElementTree

from pyqt_agi_extend import QtAgiCore

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
                skill_class_ref = QtAgiCore.QAgiPackages.__pkg__(in_pkg).__module__(in_module).__import__(skill_class)
                return skill_class_ref
            except Exception as ex:
                rospy.logerr('Import fail from Skill "%s" !'%name)
                rospy.logerr(ex)
                return None
        else:
            rospy.logerr('Skill named "%s" no found in register file !'%name)
            return None
    
    
    
