#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : accounts.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy
import os
import base64
from xml.etree import ElementTree
from python_qt_binding.QtCore import *

from user import User
from privilege import Privilege

from cobot_gui.util import CobotGuiException
from cobot_gui.res import R

def xml_to_db_file():
    
    xmlstr = """<?xml version='1.0' encoding='utf8'?>
<accounts>
    <user id="mmn">
        <created>22-05-2014</created>
        <modified>23-05-2014</modified>
        <privilege>developer</privilege>
        <password>YXRpMDA2</password>
    </user>
    <user id="martin">
        <created>22-05-2014</created>
        <modified>23-05-2014</modified>
        <privilege>operator</privilege>
        <password>YXRpMDA2</password>
    </user>
</accounts>"""
    
    xmlencode = base64.encodestring(xmlstr)
    
    with open(R.accounts.dir+'/accounts_init.db','w') as f_db:
        f_db.write(xmlencode)
        
def root_xml_to_db_file():
    
    xmlstr = """<?xml version='1.0' encoding='utf8'?>
<accounts>
</accounts>"""
    
    xmlencode = base64.encodestring(xmlstr)
    
    with open(R.accounts.dir+'/accounts_root.db','w') as f_db:
        f_db.write(xmlencode)
        
        
def indent(elem, level=0):
    
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

setattr(ElementTree, 'indent', indent)

## @package: accounts
##
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 22/05/2014

## @class UserAccounts
## @brief Manage user accounts file
class UserAccounts:
    """ Manage user account file xml:
        - Get user list
        - Find user account,
        - Add user account,
        - Modif user account,
        - Remove user account.
    """
    
    ACCOUNTS_FILENAME = 'accounts.db'
    
    #Primary keys
    USER      = 'user'
    PRIVILEGE = 'privilege'
    PASSWORD  = 'password'
    #Keys
    UID      = 'id'
    CREATED  = 'created'
    MODIFIED = 'modified'
    
    USER_STR_ITEM = """<user id="%s">
                         <created>%s</created>
                         <modified>%s</modified>
                         <privilege>%s</privilege>
                         <password>%s</password>
                       </user>"""
    
    def __init__(self, context=None):
        """Constructor"""
        
        self._context = context
        
        self.accounts_dir = R.accounts.dir
        
        with open(os.path.join(self.accounts_dir,self.ACCOUNTS_FILENAME),'r') as \
             file_encode:
            accounts_encode = file_encode.read()
        
        try:
            accounts_decode = base64.decodestring(accounts_encode)
        except Exception as e:
            raise CobotGuiException('The user accounts file is corrupted "%s"!'%str(e))
        
        self.accounts_xml = None
        
        try:
            self.accounts_xml = ElementTree.fromstring(accounts_decode)
        except Exception as e:
            raise CobotGuiException('UserAccountsManager.__init__() raised with exception "%s"'%e)
        finally:
            self._xml_file_generator()
            
    
    def update(self):
        
        xmlstr = ElementTree.tostring(self.accounts_xml,
                                      encoding='utf8',
                                      method='xml')
        
        xmlencode = base64.encodestring(xmlstr)
        
        with open(os.path.join(self.accounts_dir,self.ACCOUNTS_FILENAME),'w') as \
             f_accounts:
            f_accounts.write(xmlencode)
            
        from shutil import copyfile
        #Create backup file for the rescue mode
        copyfile(os.path.join(self.accounts_dir,self.ACCOUNTS_FILENAME),
                 os.path.join(self.accounts_dir,'backup','accounts_back.db'))
        
        self._xml_file_generator()
        
    def _xml_file_generator(self):
        
        xmlstr = ElementTree.tostring(self.accounts_xml,
                                      encoding='utf8',
                                      method='xml')
        with open(os.path.join(self.accounts_dir,'accounts.xml'),'w') as \
             f_accounts_xml:
            f_accounts_xml.write(xmlstr)
    
    def resolve_path(self, userid):
        return './%s[@%s="%s"]'%(self.USER, self.UID, userid)
     
    def user_list(self):
        """Read and get user(s) id list registered in user accounts file
        @return: user_list: user(s) id list.
        @type user_list: array string.
        """
        user_list = []
        
        for user in self.accounts_xml:
            user_list.append(user.attrib[self.UID])
             
        return user_list
        
    def find(self, userid):
        """Read and get user account information
          
        @param: userid: user id.
        @type userid: str.
          
        @return: userinfo: user informations.
        @type userinfo: C{User}.
        """
        
        user_account = self.accounts_xml.find(self.resolve_path(userid))
        
        if user_account is None:
            rospy.logerr('User "%s" not found !'%userid)
            return None
        
        userinfo = User()
        userinfo.userid    = userid
        userinfo.created   = user_account.find(self.CREATED).text
        userinfo.modified  = user_account.find(self.MODIFIED).text
        userinfo.privilege = Privilege.TOLEVEL[user_account.find(self.PRIVILEGE).text]
        userinfo.password  = user_account.find(self.PASSWORD).text
        userinfo.encoded   = True
        
        return userinfo
        
    def add(self, userinfo):
        """Add new user account in "accounts.db" file.
        
        @param: userinfo: user informations.
        @type userinfo: C{User}.
        """
        
        user_account = self.accounts_xml.find(self.resolve_path(userinfo.userid))
        
        if user_account is not None:
            raise CobotGuiException('Do not add the user id "%s" is already used !'
                                     %userinfo.userid)
        
        user_str = self.USER_STR_ITEM%(userinfo.userid,
                                       str(rospy.get_rostime()),
                                       str(rospy.get_rostime()),
                                       Privilege.TOSTR[userinfo.privilege],
                                       userinfo.password)
        
        try:
            
            user_xml = ElementTree.fromstring(user_str)
            self.accounts_xml.append(user_xml)
            ElementTree.indent(self.accounts_xml)
            self.update()
            
        except Exception as e:
            raise CobotGuiException('Do not add the user id "%s" because %s !'
                                     %(userinfo.userid, str(e)))
    
    def modif(self, usersource, usermodifed):
        """Update user informations.
        
        @param: usersource: current user informations.
        @type usersource: C{User}.
        
        @param: usermodifed: new user informations.
        @type usermodifed: C{User}.
        
        """
        
        if usersource.userid != usermodifed.userid:
            raise CobotGuiException("Change user id not allowed !")
        
        user_account = self.accounts_xml.find(self.resolve_path(usersource.userid))
        
        if user_account is None:
            raise CobotGuiException('Invalid user id "%s" is not found !'
                                    %usersource.userid)
        
        if usersource.password != user_account.find(self.PASSWORD).text:
            raise CobotGuiException('Invalid password from user id "%s" !'
                                    %usersource.userid)
        else:
            user_account.find(self.MODIFIED).text  = str(rospy.get_rostime())
            user_account.find(self.PASSWORD).text  = usermodifed.password
            user_account.find(self.PRIVILEGE).text = Privilege.TOSTR[usermodifed.privilege]
        
        try:
            self.update()
        except Exception as e:
            raise CobotGuiException(str(e))
         
    def remove(self, userinfo):
        """Remove user account.
        
        @param: userinfo: user informations.
        @type userinfo: C{User}.
        """
        
        user_account = self.accounts_xml.find(self.resolve_path(userinfo.userid))
         
        try:
            self.accounts_xml.remove(user_account)
        except:
            raise CobotGuiException('Connot removed user id "%s" is not registered !'
                                    %userinfo.userid)
        try:
            self.update()
        except Exception as e:
            raise CobotGuiException(str(e))
        
if __name__ == '__main__':
    
    print Privilege.LEVEL[Privilege.NONE]
    print Privilege.LEVEL[Privilege.OPERATOR]
    print Privilege.LEVEL[Privilege.MAINTENANCE]
    print Privilege.LEVEL[Privilege.EXPERT]
    print Privilege.LEVEL[Privilege.DEVELOPER]
    
#End of file

