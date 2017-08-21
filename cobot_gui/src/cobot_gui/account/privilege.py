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


## @class Privilege
## @brief Class for difine different levels of user access.
class Privilege:
    
    NONE        = -1
    OPERATOR    = 0
    MAINTENANCE = 1
    EXPERT      = 2
    DEVELOPER   = 3
    
    TOSTR = {NONE       : 'none',
             OPERATOR   : 'operator',
             MAINTENANCE: 'maintenance',
             EXPERT     : 'expert',
             DEVELOPER  : 'developer'}
    
    TOLEVEL = {'none'       : NONE,
               'operator'   : OPERATOR,
               'maintenance': MAINTENANCE,
               'expert'     : EXPERT,
               'developer'  : DEVELOPER}
    
#End of file

