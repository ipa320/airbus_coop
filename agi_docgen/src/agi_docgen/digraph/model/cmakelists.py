#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : build.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

from agi_docgen.digraph.digraph import *

# def getStdTRModel(topic, msg, bgcolor=RgbColor.White, align = ALIGN.Left):
#     
#     model = TR()
#     
#     topic_td = TD()
#     topic_td.setAttrib(TD.COLSPAN, 1)
#     topic_td.setAttrib(TD.ALIGN, align)
#     topic_td.setAttrib(TD.BGCOLOR, bgcolor)
#     topic_td.setText(topic)
#     model.addTD(topic_td)
#     
#     msg_td = TD()
#     msg_td.setAttrib(TD.COLSPAN, 1)
#     msg_td.setAttrib(TD.ALIGN, align)
#     msg_td.setAttrib(TD.BGCOLOR, bgcolor)
#     msg_td.setText(msg)
#     model.addTD(msg_td)
#     
#     return model

def getStdTDModel(text, bgcolor=RgbColor.White, align = ALIGN.Center):
    
    model = TD()
    model.setAttrib(TD.COLSPAN, 1)
    model.setAttrib(TD.ALIGN, align)
    model.setAttrib(TD.BGCOLOR, bgcolor)
    model.setText(text)
    
    return model

class _CMakeListsDotModel(NODE):
    
    def __init__(self, node_name, node_title, title_color=RgbColor.LightSkyBlue):
        NODE.__init__(self, node_name)
        
        self._table = TABLE()
        self._table.setAttrib(TABLE.BORDER, 0)
        self._table.setAttrib(TABLE.CELLBORDER, 1)
        self._table.setAttrib(TABLE.CELLSPACING, 0)
        self._table.setAttrib(TABLE.BGCOLOR, RgbColor.White)
        
        title = TD()
        title.setAttrib(TD.ALIGN, ALIGN.Center)
        title.setAttrib(TD.BGCOLOR, title_color)
        title.setAttrib(TD.COLSPAN, 1)
        title.setText(node_title)
        self._table.addTR(TR(title))
        
    def add(self, name):
        self._table.addTR(TR(getStdTDModel(name)))
        
    def getNode(self):
        self.setHtml(self._table)
        return self
    
    def __str__(self):
        self.setHtml(self._table)
        return NODE.__str__(self)
    
class BuildDependDotModel(_CMakeListsDotModel):
    def __init__(self):
        _CMakeListsDotModel.__init__(self,
                                     'find_package',
                                     "Package dependencies",
                                     RgbColor.Red)
#         self.setAttrib(NODE.SHAPE, "folder")
        
class AddMessageFilesDotModel(_CMakeListsDotModel):
    def __init__(self):
        _CMakeListsDotModel.__init__(self,
                                     'add_message_files',
                                     "Message files generated",
                                     RgbColor.LightSkyBlue)
        
class AddServiceFilesDotModel(_CMakeListsDotModel):
    def __init__(self):
        _CMakeListsDotModel.__init__(self,
                                      'add_service_files',
                                      "Service files generated",
                                      RgbColor.Yellow)
        
class AddActionFilesDotModel(_CMakeListsDotModel):
    def __init__(self):
        _CMakeListsDotModel.__init__(self,
                                     'add_action_files',
                                     "Action files generated",
                                     RgbColor.Chartreuse)
        
class AddExecutableDotModel(_CMakeListsDotModel):
    def __init__(self, name='add_executable', label='Executable generated'):
        _CMakeListsDotModel.__init__(self,
                                     name,
                                     label,
                                     RgbColor.OrangeRed)
        
        
class AddLibraryDotModel(_CMakeListsDotModel):
    def __init__(self):
        _CMakeListsDotModel.__init__(self, 
                                     'add_library',
                                     "Library generated",
                                     RgbColor.LightCoral)

# TFs Model

if __name__ == '__main__':

    digraph = Digraph("digraph_test")
    
    nconf = NODE("node")
    nconf.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
    digraph.addNode(nconf)
    
    pkg = NODE("ros_package")
    pkg.setAttrib(NODE.SHAPE, SHAPE.Ellipse)
    pkg.setAttrib(NODE.STYLE, STYLE.FILLED)
    pkg.setAttrib(NODE.COLOR, RgbColor.CornflowerBlue)
    pkg.setAttrib(NODE.FONTSIZE, 22)
    digraph.addRootNode(pkg)
    
    msg = AddMessageFilesDotModel()
    
    msg.add("Twist1")
    msg.add("Twist2")
    msg.add("Twist3")
    msg.add("Twist4")
    
    digraph.addNode(msg)
    digraph.connect(digraph.getRootNode(), msg)
    
    srv = AddServiceFilesDotModel()
    
    srv.add("TwistSrv1")
    srv.add("TwistSrv2")
    srv.add("TwistSrv3")
    srv.add("TwistSrv4")
    
    digraph.addNode(srv)
    digraph.connect(digraph.getRootNode(), srv)
    
    action = AddActionFilesDotModel()
    
    action.add("TwistSrv1")
    action.add("TwistSrv2")
    action.add("TwistSrv3")
    action.add("TwistSrv4")
    
    digraph.addNode(action)
    digraph.connect(digraph.getRootNode(), action)
    
    lib = AddLibraryDotModel()
    
    lib.add("TwistSrv1")
    lib.add("TwistSrv2")
    lib.add("TwistSrv3")
    lib.add("TwistSrv4")
    
    digraph.addNode(lib)
    digraph.connect(digraph.getRootNode(), lib)
    
    ex = AddExecutableDotModel()
    
    ex.add("TwistSrv1")
    ex.add("TwistSrv2")
    ex.add("TwistSrv3")
    ex.add("TwistSrv4")
    
    digraph.addNode(ex)
    digraph.connect(digraph.getRootNode(), ex)
    
    digraph.saveDot("/home/ihm-pma/Documents/dot_test/CmakeListsModel.dot")
    digraph.dotToPng("/home/ihm-pma/Documents/dot_test/CmakeListsModel.png")

    
    