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

class NodeDescription(HtmlElement):
    
    def __init__(self):
        HtmlElement.__init__(self, tag=html.Grouping.p)
        
    def read(self, node_name, node_xml):
        
        try:
            self.append(node_xml.find("description"))
        except:
            try:
                self.text = node_xml.find("description").text
#                 self.text.replace("$")
            except:
                return False
            
        return True
    
        