#!/usr/bin/env python
#
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


import os
import sys
import rospy
import rospkg

from agi_docgen.common import html
from agi_docgen.common.html import HtmlElementTree
from agi_docgen import env
from agi_docgen.docgen.menu import Menu
import catkin_pkg.workspaces

class AbstractIndex(HtmlElementTree):
    
    def __init__(self):
        HtmlElementTree.__init__(self,
                    html.loadHtml(os.path.join(env.ROSDOC_TEMPLATES,'index.html')))
        
        self._header_item  = self.getroot().find("./body/div/header")
        self._nav_item     = self.getroot().find("./body/div/nav")
        
    def setNavigation(self, nav):
        self._nav_item.append(nav)
        
    def save(self):
        html.indent(self.getroot())
        self.write(os.path.join(env.ROSDOC_GEN, "index.html"),
                   encoding="utf8",
                   method="xml")
    
    def __str__(self):
        html.indent(self.getroot())
        return html.tostring(self.getroot())
    
class Index(AbstractIndex):
    os.system("cp -r "+env.ROSDOC_ROOT+"/resources/* %s"%env.OUTPUT)
    os.system("mkdir -p "+env.ROSDOC_DOT+"/gen")
    def __init__(self, dir_path):
        AbstractIndex.__init__(self)
        
        self._menu = Menu()
        self._menu.parse(dir_path)
        self.setNavigation(self._menu)
        # Generate all html documentations
        self._menu.generate(self)

if __name__ == '__main__':
  rospy.init_node('index')

  rospy.loginfo("Output path: %s"%env.ROSDOC_GEN)

  if rospy.has_param('/agi_docgen/pkg_dir'):
    ROS_PKG = rospy.get_param('/agi_docgen/pkg_dir')
    rospack = rospkg.RosPack()
    try:
      ROS_WS = rospack.get_path(ROS_PKG)
      dir_path = ROS_WS
      rospy.loginfo("Generating documentation for the package: %s"%ROS_PKG)
    except:
      ROS_WS = ROS_PKG
      dir_path = ROS_WS + "/src"
      rospy.loginfo("Generationg documentation for the workspace: %s"%ROS_PKG)
      if not os.path.isdir(ROS_PKG+"/src"):
        rospy.logerr("Please define a valid workspace, %s doesn't contain a workspace"%ROS_WS)
        sys.exit(0)
  else:
    ROS_WSs = catkin_pkg.workspaces.get_spaces()
    ROS_WS = os.path.dirname(ROS_WSs[0])
    dir_path = ROS_WS + "/src"
    rospy.loginfo("Generationg documentation for the current workspace: %s"%ROS_WS)
    sys.path.append(os.path.join(ROS_WS,'src'))
    sys.path.append(os.path.join(env.ROSDOC_ROOT,'scripts'))

  index = Index(dir_path)
  index.save()


