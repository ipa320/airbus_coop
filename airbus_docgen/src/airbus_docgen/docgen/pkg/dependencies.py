#!/usr/bin/env python
#
# Copyright 2015 Airbus
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

from airbus_docgen.common import html
from airbus_docgen.common.html import HtmlElement

class PackageDependencies(HtmlElement):
    
    def __init__(self, pkgdir, pkg_xml):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"pkg-dep"})
        
        self._dep_lists = []
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of first order '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org/catkin/package.xml")
        dep_href.set("target", "_blank")
        dep_href.text = "package dependencies :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        
        for dep in pkg_xml.iter("build_depend"):
            li = HtmlElement(html.Grouping.li)
            li.text = dep.text
            self._dep_lists.append(dep.text)
            ul.append(li)
        
        p.append(ul)
        self.append(p)
        
    def get_dependencies_lists(self):
        return self._dep_lists
        
