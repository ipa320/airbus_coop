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

