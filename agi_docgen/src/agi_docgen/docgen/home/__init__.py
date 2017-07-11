#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : launch.py
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

from agi_docgen.docgen.home.workspace import RosWorkspace

class HtmlHomeFileGenerator(HtmlElementTree):
    
    def __init__(self, index, packages_dir):
        
        HtmlElementTree.__init__(self, index.getroot())
        
        div = self.getroot().find("./body/div")
        
        section = HtmlElement(html.Sections.section)
        #{
        
        title = HtmlElement(html.Sections.h2)
        title.text = "1. Overview"
        section.append(title)
        
        aros_title = HtmlElement(html.Sections.h3)
        aros_title.text = "1.1 About ROS"
        section.append(aros_title)
        
        aros_article = HtmlElement(html.Sections.article)
        aros_article.text = """The Robot Operating System (ROS) is a flexible framework for writing robot software.
           It is a collection of tools,
           libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms."""
        section.append(aros_article)
        
        ws_title = HtmlElement(html.Sections.h3)
        ws_title.text = "1.2 AGI ROS Workspace"
        section.append(ws_title)
        
        try:
            section.append(RosWorkspace(packages_dir))
        except Exception as ex:
            html.HTMLException(ex, section)
        
        dev_title = HtmlElement(html.Sections.h3)
        dev_title.text = "1.3 Team"
        section.append(dev_title)
        
        ref_title = HtmlElement(html.Sections.h3)
        ref_title.text = "1.4 References"
        section.append(ref_title)
        
        #}
        
        div.append(section)

    def save(self):
        html.indent(self.getroot())
        self.write(os.path.join(env.ROSDOC_GEN, "home.html"),
                   encoding="utf8",
                   method="xml")
    
    def __str__(self):
        html.indent(self.getroot())
        return html.tostring(self.getroot())
    