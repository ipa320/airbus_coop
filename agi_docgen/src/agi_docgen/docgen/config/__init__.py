#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : config.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
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