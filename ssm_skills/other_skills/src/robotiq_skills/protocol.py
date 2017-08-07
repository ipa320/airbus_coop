#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : protocol.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
#Robotiq status
ROBOTIQ_PENDING         = 0
ROBOTIQ_RUNNING         = 11
ROBOTIQ_ACTIVE          = 1
ROBOTIQ_PREEMPTED       = 2
ROBOTIQ_SUCCEEDED       = 3
ROBOTIQ_ABORTED         = 4
ROBOTIQ_REJECTED        = 5
ROBOTIQ_PREEMPTING      = 6
ROBOTIQ_RECALLING       = 7
ROBOTIQ_RECALLED        = 8
ROBOTIQ_LOST            = 9
ROBOTIQ_TIMEOUT         = 10

ROBOTIQ_ERR_CODE = [ROBOTIQ_PREEMPTED,
                    ROBOTIQ_ABORTED,
                    ROBOTIQ_REJECTED,
                    ROBOTIQ_PREEMPTING,
                    ROBOTIQ_RECALLING,
                    ROBOTIQ_RECALLED,
                    ROBOTIQ_LOST,
                    ROBOTIQ_TIMEOUT]

#Robotiq command value
OPENNED = 0
CLOSED  = 255

CMD_OPEN  = 0
CMD_CLOSE = 255

FINGER_OPENNED = 3
FINGER_GRASPED = 2
FINGER_CLOSED  = 3

#End of file
