#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : custom_rosnode.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import time
import rospy
import rosnode


def rosnode_ping_all(verbose=False):
    """
    Ping all running nodes
    @return [str], [str]: pinged nodes, un-pingable nodes
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosnode.rosgraph.Master(rosnode.ID)
    try:
        state = master.getSystemState()
    except rosnode.socket.error:
        raise rosnode.ROSNodeIOException("Unable to communicate with master!")

    nodes = []
    for s in state:
        for t, l in s:
            nodes.extend(l)
    nodes = list(set(nodes)) #uniq
    if verbose:
        print("Will ping the following nodes: \n"+''.join([" * %s\n"%n for n in nodes]))
    pinged = {}
    unpinged = []
    for node in nodes:
        start = time.time()
        status = rosnode.rosnode_ping(node, max_count=1, verbose=verbose)
        end = time.time()
        dur = (end-start)*1000.
        if status:
            pinged.update({node:dur})
        else:
            unpinged.append(node)
        
    return pinged, unpinged

def rosnode_cleanup():
    """
    This is a semi-hidden routine for cleaning up stale node
    registration information on the ROS Master. The intent is to
    remove this method once Master TTLs are properly implemented.
    """
    pinged, unpinged = rosnode_ping_all()
    if unpinged:
        master = rosnode.rosgraph.Master(rosnode.ID)
        rosnode.cleanup_master_blacklist(master, unpinged)
        
    return pinged, unpinged
