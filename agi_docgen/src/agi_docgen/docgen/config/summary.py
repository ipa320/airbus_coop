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

from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement

class ConfigSummary(HtmlElement):
    
    def __init__(self, pkgdir, pkg_xml):
        HtmlElement.__init__(self,
                             tag=html.Grouping.ul)
        
        version = HtmlElement(html.Grouping.li)
        version.text = "Version : %s"%pkg_xml.find("./version").text
        self.append(version)
        
        mtr = pkg_xml.find("./maintainer")
        
        maintainer_li = HtmlElement(html.Grouping.li)
        maintainer_li.text = "Maintainer : "
        maintainer = HtmlElement(html.Text.a)
        try:
            maintainer.set(html.Attrib.href, "mailto:%s"%mtr.attrib['email'])
        except:
            pass
        maintainer.text = mtr.text
        maintainer_li.append(maintainer)
        self.append(maintainer_li)
        
        llicense = HtmlElement(html.Grouping.li)
        llicense.text = "License : %s"%pkg_xml.find("./license").text
        self.append(llicense)
        
        if pkg_xml.find("./url") is not None:
            li = HtmlElement(html.Grouping.li)
            li.text = "Link : "
            url = HtmlElement(html.Text.a)
            url.set("href",pkg_xml.find("./url").text)
            url.set("target", "_blank")
            url.text  = pkg_xml.find("./url").text
            li.append(url)
            self.append(li)
            
        if pkg_xml.find("./export/rosdoc") is not None:
            li = HtmlElement(html.Grouping.li)
            li.text = "Sources : "
            doxygen = HtmlElement(html.Text.a)
            ref = pkgdir+"/doc/html/index.html"
            doxygen.set("href", ref)
            doxygen.set("target", "_blank")
            doxygen.text = "doxygen"
            li.append(doxygen)
            self.append(li)
            