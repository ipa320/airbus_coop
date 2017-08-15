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
from agi_docgen import env
from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement, HtmlElementTree

from agi_docgen.digraph.digraph import *

from agi_docgen.docgen.config.launch import ConfigLaunch

class HtmlConfigFileGenerator(HtmlElementTree):
    
    def __init__(self, index, launch_file):
        
        HtmlElementTree.__init__(self, index.getroot())
        
        self._filename = launch_file.split("/")[-1]
        
        div = self.getroot().find("./body/div")
        
        section = HtmlElement(html.Sections.section)
        #{
        
        title = HtmlElement(html.Sections.h1)
        title.text = self._filename.replace(".launch","")
        section.append(title)
        
        diagram_title = HtmlElement(html.Sections.h2)
        diagram_title.text = "1. Launch diagram"
        section.append(diagram_title)
        
        roslaunch = HtmlElement(html.Grouping.p)
        roslaunch.text = """roslaunch is a tool for easily launching multiple ROS nodes locally and remotely via SSH,
         as well as setting parameters on the Parameter Server.
         It includes options to automatically respawn processes that have already died.
         roslaunch takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch,
         as well as the machines that they should be run on."""
        section.append(roslaunch)
        
        try:
            launch_dot_generator = ConfigLaunch()
            launch_dot_generator.read(launch_file)
            section.append(launch_dot_generator)
        except Exception as ex:
            html.HTMLException(ex, section)
        #}
        
        div.append(section)
        
    def save(self):
        html.indent(self.getroot())
        print self._filename
        self.write(os.path.join(env.ROSDOC_GEN,
                                self._filename.replace(".launch",".html")),
                   encoding="utf8",
                   method="xml")
    
    def __str__(self):
        html.indent(self.getroot())
        return html.tostring(self.getroot())