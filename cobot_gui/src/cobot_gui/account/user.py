#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : user.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

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

