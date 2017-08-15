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


class Parameters():
    
    def __init__(self):
        self._params = {}
        
    def putParam(self, name, values):
        
        if not self._params.has_key(name):
            self._params.update({name:values})
        else:
            self._params[name] = values
        
    def getParam(self, name, default=None):
        
        if self._params.has_key(name):
            return self._params[name]
        elif default is not None:
            return default
        
        return None
    
    def __str__(self):
        
        params_str = "Parameters :\n"
        
        for k,v in self._params.items():
            params_str+= "  - %s : %s\n"%(k,str(v))
            
        return params_str
    
    