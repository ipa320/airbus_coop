#!/usr/bin/env python

import math
import numpy

def Format44quat(pose = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]):
    px_ = pose[0]
    py_ = pose[1]
    pz_ = pose[2]
    qx_ = pose[3]
    qy_ = pose[4]
    qz_ = pose[5]
    qw_ = pose[6]  
        
    Mat = numpy.eye(4) ##Init
    Mat_ = QuatToMatrix(qx_, qy_, qz_, qw_)
    Mat[0][0] = Mat_[0][0]
    Mat[1][0] = Mat_[1][0]
    Mat[2][0] = Mat_[2][0]
    Mat[0][1] = Mat_[0][1]
    Mat[1][1] = Mat_[1][1]
    Mat[2][1] = Mat_[2][1]
    Mat[0][2] = Mat_[0][2]
    Mat[1][2] = Mat_[1][2]
    Mat[2][2] = Mat_[2][2]
    Mat[0][3] = px_
    Mat[1][3] = py_
    Mat[2][3] = pz_
    
    return Mat

def Format44n(type ="deg", pose = [0.0,0.0,0.0,0.0,0.0,0.0]):
    px_ = pose[0]
    py_ = pose[1]
    pz_ = pose[2]
    A = pose[3]
    B = pose[4]
    C = pose[5] 
    
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
        
    Mat = numpy.eye(4) ##Init
    
    Mat[0][0] = math.cos(y_) * math.cos(z_)                                             ##CyCz
    Mat[0][1] = math.cos(z_)* math.sin(x_) * math.sin(y_) - math.cos(x_) * math.sin(z_) ##CzSxSy - CxSz
    Mat[0][2] = math.cos(x_)* math.cos(z_) * math.sin(y_) + math.sin(x_) * math.sin(z_) ##CxCzSy + SxSz
    
    Mat[1][0] = math.cos(y_) * math.sin(z_)                                             ##CyCz
    Mat[1][1] = math.cos(x_)* math.cos(z_) + math.sin(x_) * math.sin(y_) * math.sin(z_) ##CxCz + SxSySz
    Mat[1][2] =-math.cos(z_)* math.sin(x_) + math.cos(x_) * math.sin(y_) * math.sin(z_) ##-CzSx+ CxSySz
    
    Mat[2][0] = -math.sin(y_)                                                           ##-Sy
    Mat[2][1] = math.cos(y_)* math.sin(x_)                                              ##CySx
    Mat[2][2] = math.cos(x_)* math.cos(y_)                                              ##CxCy
    
    Mat[0][3] = px_
    Mat[1][3] = py_
    Mat[2][3] = pz_

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

def QuatToEulerZYX(type = "deg", X = 0.0, Y = 0.0, Z = 0.0, W = 1.0):
    euler = [0.0,0.0,0.0]
    euler = MatrixToEulerZYX(type,QuatToMatrix(X,Y,Z,W))
    return euler

def CheckOrientation(quat_1, quat_2):
    if(numpy.dot(quat_1,quat_2)<0.0):
        return False
    else:
        return True
    
def InverseQuat(quat):
    quat_ = [-quat[0],-quat[1],-quat[2],-quat[3]]
    return quat_

def NormalizeQuat(quat):
    result = quat
    norm = math.sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3])
    result[0] /= norm
    result[1] /= norm
    result[2] /= norm
    result[3] /= norm
    
    return quat

def MeanOfPose(pose_list):
    quat_list=[]
    position_list=[]
    for i_pose in range (0,len(pose_list)):        
        position_list.append([pose_list[i_pose][0],pose_list[i_pose][1],pose_list[i_pose][2]])
        quat_list.append([pose_list[i_pose][3],pose_list[i_pose][4],pose_list[i_pose][5],pose_list[i_pose][6]])
    ##Mean position
    position_result=[0.0,0.0,0.0]
    for i_pose in range (0,len(position_list)):
        position_result[0]+=position_list[i_pose][0]
        position_result[1]+=position_list[i_pose][1]
        position_result[2]+=position_list[i_pose][2]
    
    position_result[0]/=len(position_list)
    position_result[1]/=len(position_list)
    position_result[2]/=len(position_list)
    
    #Mean Quaternion
    quat_result = [0.0,0.0,0.0,0.0]
    first = quat_list[0]
    for i_quat in range (1,len(quat_list)):
        if(CheckOrientation(quat_list[i_quat], first)):
            quat_list[i_quat] = InverseQuat(quat_list[i_quat])
    
    for i_quat in range (0,len(quat_list)):
        quat_result[0]+=quat_list[i_quat][0]
        quat_result[1]+=quat_list[i_quat][1]
        quat_result[2]+=quat_list[i_quat][2]
        quat_result[3]+=quat_list[i_quat][3]
        
    quat_result[0]/=len(quat_list)
    quat_result[1]/=len(quat_list)
    quat_result[2]/=len(quat_list)
    quat_result[3]/=len(quat_list)
    
    quat_result = NormalizeQuat(quat_result)
    
    pose_result = [position_result[0],position_result[1],position_result[2],quat_result[0],quat_result[1],quat_result[2],quat_result[3]]
    return pose_result

def Format44(type ="deg", pose = [0.0,0.0,0.0,0.0,0.0,0.0]):
    px_ = pose[0]
    py_ = pose[1]
    pz_ = pose[2]
    A = pose[3]
    B = pose[4]
    C = pose[5] 
    
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
        
    Mat = [[1.0,0.0,0.0,0.0], [0.0,1.0,0.0,0.0], [0.0,0.0,1.0,0.0], [0.0,0.0,0.0,1.0]] ##Init
    
    Mat[0][0] = math.cos(y_) * math.cos(z_)                                             ##CyCz
    Mat[0][1] = math.cos(z_)* math.sin(x_) * math.sin(y_) - math.cos(x_) * math.sin(z_) ##CzSxSy - CxSz
    Mat[0][2] = math.cos(x_)* math.cos(z_) * math.sin(y_) + math.sin(x_) * math.sin(z_) ##CxCzSy + SxSz
    
    Mat[1][0] = math.cos(y_) * math.sin(z_)                                             ##CyCz
    Mat[1][1] = math.cos(x_)* math.cos(z_) + math.sin(x_) * math.sin(y_) * math.sin(z_) ##CxCz + SxSySz
    Mat[1][2] =-math.cos(z_)* math.sin(x_) + math.cos(x_) * math.sin(y_) * math.sin(z_) ##-CzSx+ CxSySz
    
    Mat[2][0] = -math.sin(y_)                                                           ##-Sy
    Mat[2][1] = math.cos(y_)* math.sin(x_)                                              ##CySx
    Mat[2][2] = math.cos(x_)* math.cos(y_)                                              ##CxCy
    
    Mat[0][3] = px_
    Mat[1][3] = py_
    Mat[2][3] = pz_

    return Mat

def FormatPose(type ="deg", Mat=[]):
    pose = [0.0,0.0,0.0,0.0,0.0,0.0] ## init
    pose[0] = Mat[0][3]
    pose[1] = Mat[1][3]
    pose[2] = Mat[2][3]
    if(type == "quat"):
        pose = [0.0,0.0,0.0,0.0,0.0,0.0,1.0]
        pose[0] = Mat[0][3]
        pose[1] = Mat[1][3]
        pose[2] = Mat[2][3]
        quat = MatrixToQuat(Mat[0:3][0:3])
        pose[3] = quat[0]
        pose[4] = quat[1]
        pose[5] = quat[2]
        pose[6] = quat[3]
        
        return pose
        
        
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
        pose[3] = Z_*180.0 / math.pi
        pose[4] = Y_*180.0 / math.pi
        pose[5] = X_*180.0 / math.pi
    else:
        if(type == "rad"):
             pose[3] = Z_
             pose[4] = Y_
             pose[5] = X_
        else:
            print("[ConvertionTool] Type non recognized")
            return -1
        
    return pose

def DiffPoses(pose1, pose2):
    qpose1_ = QuatToEulerZYX("deg", pose1[3], pose1[4], pose1[5], pose1[6])
    qpose2_ = QuatToEulerZYX("deg", pose2[3], pose2[4], pose2[5], pose2[6])
    pose1_ = [pose1[0],pose1[1],pose1[2],qpose1_[0],qpose1_[1],qpose1_[2]]
    pose2_ = [pose2[0],pose2[1],pose2[2],qpose2_[0],qpose2_[1],qpose2_[2]]
    Mat_1 = Format44("deg", pose1_)
    Mat_2 = Format44("deg", pose2_)
    
    return numpy.dot(Mat_2,numpy.linalg.inv(Mat_1))
    
def Checklimit(lim_mm,pose1,pose2):
    x_ = pose1[0] - pose2[0]
    y_ = pose1[1] - pose2[1]
    z_ = pose1[2] - pose2[2]
    
    dist = math.sqrt(x_*x_ + y_*y_ + z_*z_)
    
    if(dist <lim_mm):
        return True
    else:
        return False
    
def ChecklimitJt(lim_angular,joint1,joint2):
    in_limit = True
    for i_jt in range(len(joint1)):
        diff = (joint1[i_jt])-(joint2[i_jt]) 
        if(diff > lim_angular or diff < -lim_angular):
            in_limit = False
            
    return in_limit
