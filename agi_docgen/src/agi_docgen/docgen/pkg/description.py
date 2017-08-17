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

from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement

class PackageDescription(HtmlElement):
    
    def __init__(self, pkgdir, pkg_xml):
        HtmlElement.__init__(self,
                             tag=html.Grouping.p)
        
        try:
            self.append(pkg_xml.find("./description"))
        except:
            try:
                self.text = pkg_xml.find("./description").text
            except:
                raise Exception("Connot found description tag into %s/package.xml"%pkgdir)
            
        