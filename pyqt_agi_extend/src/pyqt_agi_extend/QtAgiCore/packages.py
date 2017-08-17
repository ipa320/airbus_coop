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

import rospy
from roslib.packages import get_pkg_dir, ROSPkgException
import os
import sys
import __builtin__

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi
from roslib.packages import get_pkg_dir
from xml.etree import ElementTree

# RE
RE_PREFIXED_TAG = "$"
RE_PREFIXED_BEGIN_TAG = "${"
RE_PREFIXED_END_TAG = "}"

# Default prefix name
DEFAULT_PREFIX_NAME    = "prefix"
ROS_WS_PREFIX_NAME     = "rosws"
ROS_WS_SRC_PREFIX_NAME = "rossrc"

# Provide the path of package prefixed 
# Example:
#   - "${PAKAGE_NAME}/../../..."
# The prefixed ${PAKAGE_NAME} pkg is remplaced by get_pkg_dir(PAKAGE_NAME)
def get_pkg_dir_from_prefix(path, ros_package_path=None):
    
    if RE_PREFIXED_TAG not in path:
        return path
    
    splitpath = path.split(RE_PREFIXED_END_TAG)
    
    after_prefix_path = splitpath[-1]
    
    prefix = splitpath[0].split(RE_PREFIXED_BEGIN_TAG)[-1]
    
    if prefix == DEFAULT_PREFIX_NAME:
        if ros_package_path is not None:
            rpath = ros_package_path+after_prefix_path
        else:
            raise ROSPkgException("Unknown prefixed ${prefix} path ! not associated to ros package path !")
    elif prefix == ROS_WS_PREFIX_NAME:
        rpath = get_ros_workspace_dir()+after_prefix_path
    elif prefix == ROS_WS_SRC_PREFIX_NAME:
        print get_ros_workspace_src_dir(),after_prefix_path
        rpath = get_ros_workspace_src_dir()+after_prefix_path
    else:	
        rpath = get_pkg_dir(prefix)+after_prefix_path
    
    return rpath

def get_ros_workspace_dir():
    return os.environ["ROS_WORKSPACE"]

def get_ros_workspace_src_dir():
    return os.path.join(os.environ["ROS_WORKSPACE"],'src')

class PyModule:
    
    def __init__(self, module_name):
        
        rospy.logdebug("Try to import module %s ..."%module_name)
        
        self._module = None

        try:
            self._module = __import__(module_name)
        except Exception as ex:
            rospy.logerr(str(ex))
            raise ex

        rospy.logdebug("Module %s imported."%str(self._module)) 
    
    def __import__(self, py_class):
        rospy.logdebug("  - Try to class %s ..."%py_class) 
        class_ref = getattr(self._module, py_class, None)
        rospy.logdebug("  - Class %s imported."%str(class_ref))
        return class_ref
    
class PyPkg:
    
    def __init__(self, pkg):
        self._pkg_dir = get_pkg_dir(pkg)
    
    def __module__(self, _module):
        
        _module = _module.replace('.','/')
        
        fullpath = self._pkg_dir+"/"+_module
        
        fullpath_split  = fullpath.split('/')
        module_dir      = '/'.join(fullpath_split[:-1])
        module_name     = fullpath_split[-1]
        
        sys.path.append(str(module_dir))
        
        return PyModule(str(module_name))

class QAgiPackages:
    
    def __init__(self, pkg):
        return QAgiPackages.__pkg__(pkg)
    
    @staticmethod
    def get_pkg_dir(exp):
        return get_pkg_dir_from_prefix(exp)
    
    @staticmethod
    def getDirByName(pkg_name):
        return get_pkg_dir(pkg_name)
    
    def getRessources(self):
        return None
    
    def getRessourcesById(self, category, resid):
        return None
    
    @staticmethod
    def __ws_dir__():
        return os.environ[ROS_PACKAGE_PATH].split(':')[0]
    
    @staticmethod
    def __src_dir__():
        return os.environ[ROS_PACKAGE_PATH].split(':')[1]
    
    @staticmethod
    def __pkg__(pkg):
        return PyPkg(pkg)
    
