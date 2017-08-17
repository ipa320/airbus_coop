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

import os
import re
from agi_docgen import env
from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement, HtmlElementTree

from agi_docgen.docgen.pkg.summary import PackageSummary
from agi_docgen.docgen.pkg.description import PackageDescription
from agi_docgen.docgen.pkg.dependencies import PackageDependencies
from agi_docgen.docgen.pkg.generations import PackageGenerations
from agi_docgen.docgen.pkg.node import RosNode

class AgiDoc(HtmlElement):
    
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.section,
                             attrib={"class":"nodes"})
    
    def read(self, pkgdir, agi_xml, index):
        
        index_node=0
        
        for node_xml in agi_xml.iter('node'):
            index_node+=1
            title = HtmlElement(html.Sections.h3)
            node_name = node_xml.attrib['name']
            title.text = "%i.%i. %s"%(index, index_node, node_name)
            self.append(title)
            
            try:
                ros_node = RosNode()
                if ros_node.read(node_name, node_xml, index, index_node) is True:
                    self.append(ros_node)
            except Exception as ex:
                html.HTMLException(ex, self)
            
        if index_node is 0:
            return False
        else:
            return True

class RosPackage(HtmlElement):
    
    def __init__(self, pkgdir):
        HtmlElement.__init__(self,
                             tag=html.Sections.section,
                             attrib={"class":"package"})
        self._h2_index = 0
        self._dep_pkg = None
        pkg_xml = None
        # Load and read package.xml ressource
        pkg_xml_dir = pkgdir+'/package.xml'
        if os.access(pkg_xml_dir, os.R_OK):
            pkg_xml = html.loadHtml(pkg_xml_dir)
            self._read_pkg_xml(pkgdir, pkg_xml)
        else:
            html.HTMLException("Cannot found %s !"%pkg_xml_dir, self)
        
        # Load and read CMakeLists.txt ressource
        cmakelists_dir = pkgdir+'/CMakeLists.txt'
        if os.access(cmakelists_dir, os.R_OK):
            with open(cmakelists_dir) as fp:
                cmakelists = fp.read()
                self._read_cmakelists(pkgdir, cmakelists)
        else:
            html.HTMLException("Cannot found %s !"%cmakelists_dir, self)
            
        if pkg_xml is not None:
            self._read_agi_doc_xml(pkgdir, pkg_xml)
    
    def _read_pkg_xml(self, pkgdir, pkg_xml):
        
        pkg_name = HtmlElement(html.Sections.h1)
        pkg_name.text = pkg_xml.find("./name").text
        self.append(pkg_name)
        
        p = HtmlElement(html.Grouping.p)
        p.set("align","center")
        img = HtmlElement(html.EmbeddedContent.img)
        img.set("src","../dot/gen/%s.png"%pkg_xml.find("./name").text)
        
        p.append(img)
        self.append(p)
        
        pkg_summary_title = HtmlElement(html.Sections.h2)
        pkg_summary_title.text = "%i. Package Summary"%self.index_h2()
        self.append(pkg_summary_title)
        
        try:
            self.append(PackageSummary(pkgdir, pkg_xml))
        except Exception as ex:
            html.HTMLException(ex, self)
            
        pkg_desc_title = HtmlElement(html.Sections.h2)
        pkg_desc_title.text = "%i. Package description"%self.index_h2()
        self.append(pkg_desc_title)
        
        try:
            self.append(PackageDescription(pkgdir, pkg_xml))
        except Exception as ex:
            html.HTMLException(ex, self)
            
        pkg_dep_title = HtmlElement(html.Sections.h2)
        pkg_dep_title.text = "%i. Package dependencies"%self.index_h2()
        self.append(pkg_dep_title)
        
        try:
            self._dep_pkg = PackageDependencies(pkgdir, pkg_xml)
            self.append(self._dep_pkg)
        except Exception as ex:
            html.HTMLException(ex, self)
        
    def _read_cmakelists(self, pkgdir, cmakefile):
        
        try:
            pkg = PackageGenerations()
            dep_list = self._dep_pkg.get_dependencies_lists()
            if pkg.read(pkgdir, cmakefile, dep_list) is True:
                pkg_build_title = HtmlElement(html.Sections.h2)
                pkg_build_title.text = "%i. Package generation(s)"%self.index_h2()
                self.append(pkg_build_title)
                self.append(pkg)
        except Exception as ex:
            html.HTMLException(ex, self)
            
    def _read_agi_doc_xml(self, pkgdir, pkg_xml):
        
        agidoc_elem = pkg_xml.find("./export/agidoc")
        
        if agidoc_elem is not None:
            if 'src' in agidoc_elem.attrib:
                fdoc = os.path.join(pkgdir, agidoc_elem.attrib['src'])
                if os.path.isfile(fdoc):
                    agi = AgiDoc()
                    if agi.read(pkgdir, html.loadHtml(fdoc), self._h2_index+1) is True:
                        title = HtmlElement(html.Sections.h2)
                        title.text = "%i. More description"%self.index_h2()
                        self.append(title)
                        self.append(agi)
                else:
                    html.HTMLException("Cannot open agidoc '%s'"%fdoc, self)
        else:
            html.HTMLException("AGI documentation not found !", self)
            
    def index_h2(self):
        self._h2_index+=1
        return self._h2_index

class HtmlPkgFileGenerator(HtmlElementTree):
    
    def __init__(self, index, pkg_dir, pkg_name):
        HtmlElementTree.__init__(self, index.getroot())
        self._pkg_name = pkg_name
        
        div = self.getroot().find("./body/div")
        
        try:
            pkg = RosPackage(pkg_dir)
            div.append(pkg)
        except Exception as ex:
            html.HTMLException(ex, div)
        
    def save(self):
        html.indent(self.getroot())
        #print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",os.path.join(env.ROSDOC_ROOT, "%s.html"%self._pkg_name)
        self.write(os.path.join(env.ROSDOC_GEN, "%s.html"%self._pkg_name),
                   encoding="utf8",
                   method="xml")
    
    def __str__(self):
        html.indent(self.getroot())
        return html.tostring(self.getroot())

