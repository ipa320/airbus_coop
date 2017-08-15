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


from agi_docgen.digraph.digraph import *

class IoPackageModel(NODE):
    
    TOPICS   = "topics"
    ACTIONS  = "actions"
    SERVICES = "services"
    
    def __init__(self, node_name):
        NODE.__init__(self, node_name)
        
        table = TABLE()
        table.setAttrib(TABLE.BORDER, 0)
        table.setAttrib(TABLE.CELLBORDER, 1)
        table.setAttrib(TABLE.CELLSPACING, 0)
        table.setAttrib(TABLE.BGCOLOR, RgbColor.White)
        
        title = TD()
        title.setAttrib(TD.ALIGN, ALIGN.Center)
        title.setAttrib(TD.BGCOLOR, RgbColor.Tomato)
        title.setText(node_name)
        table.addTR(TR(title))
        
        topics = TD()
        topics.setAttrib(TD.ALIGN, ALIGN.Center)
        topics.setAttrib(TD.BGCOLOR, RgbColor.Turquoise)
        topics.setAttrib(TD.PORT, self.TOPICS)
        topics.setText("Topics")
        table.addTR(TR(topics))
        
        actions = TD()
        actions.setAttrib(TD.ALIGN, ALIGN.Center)
        actions.setAttrib(TD.BGCOLOR, RgbColor.LightGreen)
        actions.setAttrib(TD.PORT, self.ACTIONS)
        actions.setText("Actions")
        table.addTR(TR(actions))
        
        services = TD()
        services.setAttrib(TD.ALIGN, ALIGN.Center)
        services.setAttrib(TD.BGCOLOR, RgbColor.Violet)
        services.setAttrib(TD.PORT, self.SERVICES)
        services.setText("Services")
        table.addTR(TR(services))
        
        self.setHtml(table)
            
    def topics(self):
        return {self : self.TOPICS}
    
    def actions(self):
        return {self : self.ACTIONS}
    
    def services(self):
        return {self : self.SERVICES}
    
class DepPackageModel(NODE):
    
    PACKAGES      = "packages"
    EXTERNAL_LIBS = "external_libs"
    INTERNAL_LIBS = "internal_libs"
    
    def __init__(self, node_name):
        NODE.__init__(self, node_name)
        
        table = TABLE()
        table.setAttrib(TABLE.BORDER, 0)
        table.setAttrib(TABLE.CELLBORDER, 1)
        table.setAttrib(TABLE.CELLSPACING, 0)
        table.setAttrib(TABLE.BGCOLOR, RgbColor.White)
        
        title = TD()
        title.setAttrib(TD.ALIGN, ALIGN.Center)
        title.setAttrib(TD.BGCOLOR, RgbColor.Aqua)
        title.setText(node_name)
        table.addTR(TR(title))
        
        packages = TD()
        packages.setAttrib(TD.ALIGN, ALIGN.Center)
        packages.setAttrib(TD.BGCOLOR, RgbColor.White)
        packages.setAttrib(TD.PORT, self.PACKAGES)
        packages.setText("Packages")
        table.addTR(TR(packages))
        
        external_libs = TD()
        external_libs.setAttrib(TD.ALIGN, ALIGN.Center)
        external_libs.setAttrib(TD.BGCOLOR, RgbColor.White)
        external_libs.setAttrib(TD.PORT, self.EXTERNAL_LIBS)
        external_libs.setText("External libraries")
        table.addTR(TR(external_libs))
        
        internal_libs = TD()
        internal_libs.setAttrib(TD.ALIGN, ALIGN.Center)
        internal_libs.setAttrib(TD.BGCOLOR, RgbColor.White)
        internal_libs.setAttrib(TD.PORT, self.INTERNAL_LIBS)
        internal_libs.setText("Internal libraries")
        table.addTR(TR(internal_libs))
        
        self.setHtml(table)
        
    def packages(self):
        return {self : self.PACKAGES}
    
    def internalLibs(self):
        return {self : self.INTERNAL_LIBS}
    
    def externalLibs(self):
        return {self : self.EXTERNAL_LIBS}

if __name__ == '__main__':
    
    print str(IoPackageModel("iiwa_controller"))
    
    print str(DepPackageModel("iiwa_controller"))
    