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


import rospy
from privilege import Privilege
import base64

## @class User
## @brief Class for difine the user informations.
class User(object):
    
    UNKNOWN = 'Unknown'
    NONE    = UNKNOWN
    DEFAULT_LNG = 'en'
    
    def __init__(self, userid = None, privilege = Privilege.NONE):
        
        self.userid    = userid or ''
        self.created   = ''
        self.modified  = ''
        self.connected = ''
        self.privilege = privilege
        self.password  = ''
        self.encoded   = False
        self.language  = self.DEFAULT_LNG
        
        if userid is not None:
            self.connected = rospy.get_time()
        
    def init(self, userid, privilege):
        self.connected = rospy.get_time()
        self.userid    = userid
        self.privilege = privilege
        
    def setUserId(self, user_id):
        self.userid = user_id
        
    def getUserId(self):
        return self.userid
    
    def setUserPrivilege(self, privilege):
        self.privilege = privilege
    
    def getUserPrivilege(self):
        return self.privilege
    
    def setUserLanguage(self, lng):
        self.language = lng
        
    def getUserLanguage(self):
        return self.language
    
    def whenCreated(self):
        return self.created
    
    def getWhenUserModified(self):
        return self.created
    
    def getUserSinceConnected(self):
        return self.connected
        
    def setUserPassword(self, password, encode = True):
        
        if encode:
            self.password = base64.b64encode(password)
        else:
            self.password = password
            
        self.encoded = encode
        
    def getUserPassword(self, decode = False):
        
        if decode:
            return base64.b64decode(self.password)
        else:
            return self.password
        
    def __str__(self):
        return 'User[%s,%s]'%(self.userid, Privilege.TOSTR[self.privilege]) 
    
#End of file

