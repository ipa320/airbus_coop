#!/usr/bin/env python

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : robot_skills.py
# Authors : Clement Beaudoing
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Clement Beaudoing <clement.beaudoing.external@airbus.com>
#
#
################################################################################


import rospy
import math
from roslib.packages import get_pkg_dir
from xml.etree import ElementTree

def EulerZYXToMatrix(type = "deg", A = 0.0, B = 0.0, C = 0.0):
    if(type == "deg"):
        z_ = A*math.pi /180.0
        y_ = B*math.pi /180.0
        x_ = C*math.pi /180.0
    else:
        if(type == "rad"):
             z_ = A
             y_ = B
             x_ = C
        else:
            print("[ConvertionTool] Type non recognized")
            return -1
        
    Mat = [[1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0]] ##Init
    
    Mat[0][0] = math.cos(y_) * math.cos(z_)                                             ##CyCz
    Mat[0][1] = math.cos(z_)* math.sin(x_) * math.sin(y_) - math.cos(x_) * math.sin(z_) ##CzSxSy - CxSz
    Mat[0][2] = math.cos(x_)* math.cos(z_) * math.sin(y_) + math.sin(x_) * math.sin(z_) ##CxCzSy + SxSz
    
    Mat[1][0] = math.cos(y_) * math.sin(z_)                                             ##CyCz
    Mat[1][1] = math.cos(x_)* math.cos(z_) + math.sin(x_) * math.sin(y_) * math.sin(z_) ##CxCz + SxSySz
    Mat[1][2] =-math.cos(z_)* math.sin(x_) + math.cos(x_) * math.sin(y_) * math.sin(z_) ##-CzSx+ CxSySz
    
    Mat[2][0] = -math.sin(y_)                                                           ##-Sy
    Mat[2][1] = math.cos(y_)* math.sin(x_)                                              ##CySx
    Mat[2][2] = math.cos(x_)* math.cos(y_)                                              ##CxCy
    
    return Mat




def MatrixToEulerZYX(type = "deg", Mat = [[1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0]]):
    
    euler = [0.0,0.0,0.0] ## init
    
    if(Mat[2][0] < 1):
        if(Mat[2][0] > -1):
            Y_ = math.asin(-Mat[2][0])
            Z_ = math.atan2(Mat[1][0], Mat[0][0])
            X_ = math.atan2(Mat[2][1], Mat[2][2])
        else: ## Mat[2][0] = -1
            Y_ = math.pi/2
            Z_ = -math.atan2(Mat[1][2], Mat[1][1])
            X_ = 0.0
    else: ##Mat[2][0] = 1
        Y_ = -math.pi/2
        Z_ = math.atan2(-Mat[1][2],Mat[1][1])
        X_ = 0.0
    
    if(type == "deg"):
        euler[0] = Z_*180.0 / math.pi
        euler[1] = Y_*180.0 / math.pi
        euler[2] = X_*180.0 / math.pi
    else:
        if(type == "rad"):
             euler[0] = Z_
             euler[1] = Y_
             euler[2] = X_
        else:
            print("[ConvertionTool] Type non recognized")
            return -1
        
    return euler
    


def MatrixToQuat(Mat = [[1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0]]):
    quat = [0.0,0.0,0.0,1.0] ##init
    
    tr_ = Mat[0][0] + Mat[1][1] + Mat[2][2] ##trace of the matrix
    if(tr_ > 0) :
        s = math.sqrt(tr_+1.0) * 2 ## s = 4 * qw 
        qw_ = s/4
        qx_ = (Mat[2][1] - Mat[1][2])/s
        qy_ = (Mat[0][2] - Mat[2][0])/s
        qz_ = (Mat[1][0] - Mat[0][1])/s
    else:
        if((Mat[0][0] > Mat[1][1]) and (Mat[0][0] > Mat[2][2])):
            s = 2*math.sqrt(1.0 + Mat[0][0] - Mat[1][1] - Mat[2][2])## s = 4*qx
            qw_ = (Mat[2][1] - Mat[1][2])/s
            qx_ = s/4
            qy_ = (Mat[1][0] + Mat[0][1])/s
            qz_ = (Mat[0][2] + Mat[2][0])/s
        else:
            if(Mat[1][1] > Mat[2][2]):
                s = 2*math.sqrt(1.0 - Mat[0][0] + Mat[1][1] - Mat[2][2])## s = 4*qy
                qw_ = (Mat[0][2] - Mat[2][0])/s
                qx_ = (Mat[1][0] + Mat[0][1])/s
                qy_ = s/4
                qz_ = (Mat[2][1] + Mat[1][2])/s
            else:
                s = 2*math.sqrt(1.0 - Mat[0][0] - Mat[1][1] + Mat[2][2])## s = 4*qz
                qw_ = (Mat[1][0] - Mat[0][1])/s
                qx_ = (Mat[0][2] + Mat[2][0])/s
                qy_ = (Mat[2][1] + Mat[1][2])/s
                qz_ = s/4
                
    norm = math.sqrt(qx_*qx_ + qy_*qy_ + qz_*qz_ + qw_*qw_)
    qx_ /= norm
    qy_ /= norm
    qz_ /= norm
    qw_ /= norm
    quat = [qx_, qy_, qz_, qw_]
    
    return quat


def QuatToMatrix(X = 0.0, Y = 0.0, Z = 0.0, W = 1.0):
    
    Mat = [[1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0]] ## Init
    
    norm = math.sqrt(X*X + Y*Y + Z*Z + W*W)##just in case
    
    qx = X / norm
    qy = Y / norm
    qz = Z / norm
    qw = W / norm
    
    Mat[0][0] = 1 - 2*qy*qy - 2*qz*qz
    Mat[0][1] = 2*qx*qy - 2*qz*qw
    Mat[0][2] = 2*qx*qz + 2*qy*qw
    
    Mat[1][0] = 2*qx*qy + 2*qz*qw
    Mat[1][1] = 1- 2*qx*qx - 2*qz*qz
    Mat[1][2] = 2*qy*qz - 2*qx*qw
    
    Mat[2][0] = 2*qx*qz - 2*qy*qw
    Mat[2][1] = 2*qy*qz + 2*qx*qw
    Mat[2][2] = 1- 2*qx*qx - 2*qy*qy
    return Mat

def EulerZYXToQuat(type = "deg", A = 0.0, B = 0.0, C = 0.0):
    quat = [0.0,0.0,0.0,1.0]
    quat = MatrixToQuat(EulerZYXToMatrix(type,A,B,C))
    return quat

def QuatToEulerZYX(type = "deg", X = 0.0, Y = 0.0, Z = 0.0, W = 1.0):
    euler = [0.0,0.0,0.0]
    euler = MatrixToEulerZYX(type,QuatToMatrix(X,Y,Z,W))
    return euler


