#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : index.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import os
import sys

from agi_docgen.common import html
from agi_docgen.common.html import HtmlElementTree
from agi_docgen import env
from agi_docgen.docgen.menu import Menu

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
    os.system("cp -r "+env.ROSDOC_ROOT+"/resources %s"%env.OUTPUT)  
    def __init__(self, workspace_dir):
        AbstractIndex.__init__(self)
        
        self._menu = Menu()
        self._menu.parse(workspace_dir)
        self.setNavigation(self._menu)
        # Generate all html documentations
        self._menu.generate(self)
        
if __name__ == '__main__':
    print env.ROSDOC_GEN
    sys.path.append(os.path.join(env.ROSDOC_ROOT,'scripts'))
    
    index = Index(os.path.join(env.ROS_WS,"src"))
    index.save()
    
    
