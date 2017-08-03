################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : account.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
from user import User
from privilege import Privilege
from accounts import UserAccounts

from accounts_ui import LoginDialog, \
                        UserAccountsWidget