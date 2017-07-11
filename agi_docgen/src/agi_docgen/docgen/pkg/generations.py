#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : generations.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import re
import os
from agi_docgen import env
from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement

from agi_docgen.digraph.digraph import *
from agi_docgen.digraph.model.cmakelists import AddMessageFilesDotModel, \
                                             AddActionFilesDotModel, \
                                             AddServiceFilesDotModel, \
                                             AddLibraryDotModel, \
                                             AddExecutableDotModel, \
                                             BuildDependDotModel

class MessageFilesGeneration(HtmlElement):
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"generation"})
        
    def read(self, pkgdir, f_cmake, digraph):
        
        has_msgs = False
        add_msg_dot_model = AddMessageFilesDotModel()
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of generated '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org/msg")
        dep_href.set("target", "_blank")
        dep_href.text = "message file :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        for msg in re.finditer(r"(.*?)\.msg", f_cmake):
            msfound = msg.group()
            if '#' not in msfound:
                li = HtmlElement(html.Grouping.li)
                rms = msfound.replace('FILES','').replace(' ','')
                li.text = rms
                add_msg_dot_model.add(rms)
                ul.append(li)
                has_msgs = True
        
        p.append(ul)
        self.append(p)
        
        if has_msgs is True:
            digraph.addNode(add_msg_dot_model)
            digraph.connect(digraph.getRootNode(), add_msg_dot_model)
        
        return has_msgs
        
class ActionFilesGeneration(HtmlElement):
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"generation"})
        
    def read(self, pkgdir, f_cmake, digraph):
        
        has_action = False
        add_ac_dot_model = AddActionFilesDotModel()
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of generated '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org")
        dep_href.set("target", "_blank")
        dep_href.text = "action file :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        for msg in re.finditer(r"(.*?)\.action", f_cmake):
            msfound = msg.group()
            if '#' not in msfound:
                li = HtmlElement(html.Grouping.li)
                rac = msfound.replace(' ','')
                li.text = rac
                add_ac_dot_model.add(rac)
                ul.append(li)
                has_action = True
        
        p.append(ul)
        self.append(p)
        
        if has_action is True:
            digraph.addNode(add_ac_dot_model)
            digraph.connect(digraph.getRootNode(), add_ac_dot_model)
        
        return has_action
        
class ServiceFilesGeneration(HtmlElement):
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"generation"})
        
    def read(self, pkgdir, f_cmake, digraph):
        
        has_srv = False
        add_srv_dot_model = AddServiceFilesDotModel()
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of generated '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org/srv")
        dep_href.set("target", "_blank")
        dep_href.text = "service file :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        for msg in re.finditer(r"(.*?)\.srv", f_cmake):
            msfound = msg.group()
            if '#' not in msfound:
                li = HtmlElement(html.Grouping.li)
                rsrv = msfound.replace(' ','')
                li.text = rsrv
                add_srv_dot_model.add(rsrv)
                ul.append(li)
                has_srv = True
        
        p.append(ul)
        self.append(p)
        
        if has_srv is True:
            digraph.addNode(add_srv_dot_model)
            digraph.connect(digraph.getRootNode(), add_srv_dot_model)
        
        return has_srv
        
class LibrarieGeneration(HtmlElement):
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"generation"})
    def read(self, pkgdir, f_cmake, digraph):
        
        has_lib = False
        add_lib_dot_model = AddLibraryDotModel()
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of generated '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org/catkin/CMakeLists.txt")
        dep_href.set("target", "_blank")
        dep_href.text = "libraries :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        
        for lib in re.finditer("(.*?)add_library\((?P<lib>(.+?){1,})", f_cmake):
            libfound = lib.group()
            if '#' not in libfound:
                li = HtmlElement(html.Grouping.li)
                rlib = lib.group('lib').split('/')[0].split(' ')[0]
                li.text = rlib
                add_lib_dot_model.add(rlib)
                ul.append(li)
                has_lib = True
        
        p.append(ul)
        self.append(p)
        
        if has_lib is True:
            digraph.addNode(add_lib_dot_model)
            digraph.connect(digraph.getRootNode(), add_lib_dot_model)
        
        return has_lib
        
class ExecutableGeneration(HtmlElement):
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"generation"})
        
    def read(self, pkgdir, f_cmake, digraph):
        
        has_exec = False
        add_exec_dot_model = AddExecutableDotModel()
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of generated '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org/catkin/CMakeLists.txt")
        dep_href.set("target", "_blank")
        dep_href.text = "executables :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        
        for lib in re.finditer("(.*?)add_executable\((?P<exec>(.+?){1,})", f_cmake):
            libfound = lib.group()
            if '#' not in libfound:
                li = HtmlElement(html.Grouping.li)
                rexec = lib.group('exec').split('/')[0].split(' ')[0]
                li.text = rexec
                add_exec_dot_model.add(rexec)
                ul.append(li)
                has_exec = True
        
        p.append(ul)
        self.append(p)
        
        if has_exec is True:
            digraph.addNode(add_exec_dot_model)
            digraph.connect(digraph.getRootNode(), add_exec_dot_model)
        
        return has_exec
    
class PyExecutableGeneration(HtmlElement):
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"py_generation"})
        
    def read(self, pkgdir, pkg_name, digraph):
        
        has_exec = False
        add_exec_dot_model = AddExecutableDotModel("add_py_executable","Python executable generated")
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of generated '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org/catkin/CMakeLists.txt")
        dep_href.set("target", "_blank")
        dep_href.text = "python executables :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        
        fscripts = pkgdir+'/scripts'
        fnodes   = pkgdir+'/nodes'
        
        if os.path.isdir(fscripts):
            for filename in os.listdir(fscripts):
                if filename != "__init__.py":
                    li = HtmlElement(html.Grouping.li)
                    li.text = filename
                    add_exec_dot_model.add(filename)
                    ul.append(li)
                    has_exec = True
        
        if os.path.isdir(fnodes):
            for filename in os.listdir(fnodes):
                if filename != "__init__.py":
                    li = HtmlElement(html.Grouping.li)
                    li.text = filename
                    add_exec_dot_model.add(filename)
                    ul.append(li)
                    has_exec = True
        
        p.append(ul)
        self.append(p)
        
        if has_exec is True:
            digraph.addNode(add_exec_dot_model)
            digraph.connect(digraph.getRootNode(), add_exec_dot_model)
        
        return has_exec
        
class PackageGenerations(HtmlElement):
    
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"generation"})
        
    
    def _create_digraph(self, name, dep_pkg_list):
        
        digraph = Digraph("PkgGenerationGraph")
        digraph.setAttrib(Digraph.NODESEP, 0.1)
#         digraph.setAttrib(Digraph.RANKDIR, 'LR')
         
        nconf = NODE("node")
        nconf.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
        digraph.addNode(nconf)
        
        pkg = NODE(name)
        pkg.setAttrib(NODE.SHAPE, "box3d")#SHAPE.Box)
        pkg.setAttrib(NODE.STYLE, STYLE.FILLED)
        pkg.setAttrib(NODE.COLOR, RgbColor.CornflowerBlue)
        pkg.setAttrib(NODE.FONTSIZE, 22)
        digraph.addRootNode(pkg)
        
        if dep_pkg_list is not None:
            dep_dot_model = BuildDependDotModel()
            for dep in dep_pkg_list:
                dep_dot_model.add(dep)
            digraph.addNode(dep_dot_model)
            digraph.connect(dep_dot_model, digraph.getRootNode())
            
        
        return digraph
    
    def read(self, pkgdir, f_cmake, dep_pkg_list):
        
        index = 0
        pkg_name = pkgdir.split('/')[-1]
        digraph = self._create_digraph(pkg_name, dep_pkg_list)
        
#         p = HtmlElement(html.Grouping.p)
#         img = HtmlElement(html.EmbeddedContent.img)
#         img.set("src","resources/dot/gen/%s.png"%pkg_name)
#         
#         p.append(img)
#         self.append(p)
        
        try:
            msgs = MessageFilesGeneration()
            if msgs.read(pkgdir, f_cmake, digraph) is True:
                index += 1
                title = HtmlElement(html.Sections.h3)
                title.text = "4.%i Message(s)"%index
                self.append(title)
                self.append(msgs)
        except Exception as ex:
            html.HTMLException(ex,self)
        
        try:
            ac = ActionFilesGeneration()
            if ac.read(pkgdir, f_cmake, digraph) is True:
                index += 1
                title = HtmlElement(html.Sections.h3)
                title.text = "4.%i Action(s)"%index
                self.append(title)
                self.append(ac)
        except Exception as ex:
            html.HTMLException(ex,self)
        
        try:
            srv = ServiceFilesGeneration()
            if srv.read(pkgdir, f_cmake, digraph) is True:
                index += 1
                title = HtmlElement(html.Sections.h3)
                title.text = "4.%i Service(s)"%index
                self.append(title)
                self.append(srv)
        except Exception as ex:
            html.HTMLException(ex,self)
        
        try:
            lib = LibrarieGeneration()
            if lib.read(pkgdir, f_cmake, digraph) is True:
                index += 1
                title = HtmlElement(html.Sections.h3)
                title.text = "4.%i Librarie(s)"%index
                self.append(title)
                self.append(lib)
        except Exception as ex:
            html.HTMLException(ex,self)
        
        try:
            exe = ExecutableGeneration()
            if exe.read(pkgdir, f_cmake, digraph) is True:
                index += 1
                title = HtmlElement(html.Sections.h3)
                title.text = "4.%i Executable(s)"%index
                self.append(title)
                self.append(exe)
        except Exception as ex:
            html.HTMLException(ex,self)
        
        try:
            pyexe = PyExecutableGeneration()
            if pyexe.read(pkgdir, pkg_name, digraph) is True:
                index += 1
                title = HtmlElement(html.Sections.h3)
                title.text = "4.%i Python executable(s)"%index
                self.append(title)
                self.append(pyexe)
        except Exception as ex:
            html.HTMLException(ex,self)
        
        digraph.saveDot(env.ROSDOC_DOT+"/gen/%s.dot"%pkg_name)
        digraph.dotToPng(env.ROSDOC_DOT+"/gen/%s.png"%pkg_name)
        
        if index is 0:
            return False
        else:
            return True
        