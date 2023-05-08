#!/usr/bin/env python
"""
    File name: Learning-LPV-MPC.py
    Author: Eugenio Alcala
    Email: eugenio.alcala@upc.edu.edu
    Date: 09/30/2018
    Python Version: 2.7.12
"""

import os
import sys
import datetime
import rospy
import numpy as np
import scipy.io as sio
import pdb
import pickle
import matplotlib.pyplot as plt
import math 

sys.path.append(sys.path[0]+'/ControllerObject')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/data')

from lpv_mpc.msg import ECU, prediction, Racing_Info, My_Planning, simulatorStates, pos_info
from utilities import Regression, Curvature
from dataStructures import LMPCprediction, EstimatorData, PlanningData, ClosedLoopDataObj #planning data-> sub a "my points"
from PathFollowingLPVMPC import PathFollowingLPV_MPC, ABC_computation_5SV
from l4vehicle_msgs.msg import ModelPredictiveControlCommand, VehicleState
from l4planning_msgs.msg import Waypoints
from geometry_msgs.msg import Pose2D
from scipy.io import loadmat
from numpy import tan, arctan, cos, sin, pi


np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
dt          = 0.033
downsample  = 0 # plan sampled at 1ms, we use this cte to downsample it 
rate_in_msg = 1
# horizon     = 10
horizon     = rospy.get_param("/control/N")
# workspace_path = rospy.get_param("state_estimator/path")	
ws_path = "/home/marc/workspace_mod/"  

def main():


    data_raw = np.load('/home/marc/IRI Internship/project IDIADA/working/workspace_IDIADA/src/lpv_mpc/src/TrackFile10.npy')

    rospy.init_node("publishWaypoints")
    rate = rospy.Rate(30)

    pub_plan = rospy.Publisher("waypoints", Waypoints, queue_size=1)
    j = 0

    datath    = np.zeros([len(data_raw),1])
    datax     = np.zeros([len(data_raw),1])
    datay     = np.zeros([len(data_raw),1])
    datavx    = np.zeros([len(data_raw),1])
    datavy    = np.zeros([len(data_raw),1])
    datav_raw = np.zeros([len(data_raw),1])
    dataW     = np.zeros([len(data_raw),1])
    datacur   = np.zeros([len(data_raw),1])
    i = 0

    for d in data_raw:
        datav_raw[i,0] = d[0]
        datacur[i,0]   = d[1]
        dataW[i,0]    = datav_raw[i]  * datacur[i]
        i += 1

    for i in range(1,len(datav_raw)-1):

        datath[i,0] = (datath[i-1,0] + dataW[i,0]*dt)
        datavx[i,0] = datav_raw[i,0] * cos(datath[i,0])
        datavy[i,0] = datav_raw[i,0] * sin(datath[i,0])

        datax[i,0]  = datax[i-1,0] + dt*datavx[i,0]
        datay[i,0]  = datay[i-1,0] + dt*datavy[i,0]


    index = np.arange(0,len(datax),1)

    # plt.subplot(511)
    # plt.plot(datax[index],datay[index],'.')
    # plt.legend(['Track'], loc='best')
    # plt.grid()

    # plt.subplot(512) 
    # plt.plot(index,datav_raw[index],'.')
    # plt.legend(['Vel x'], loc='best')
    # plt.grid()

    # plt.subplot(513)
    # plt.plot(index,dataW[index],'.')
    # plt.legend(['vel ang'], loc='best')
    # plt.grid()

    # plt.subplot(514)
    # plt.plot(index,datath[index],'.')
    # plt.legend(['Yaw'], loc='best')
    # plt.grid()

    # plt.subplot(515)
    # plt.plot(index,datacur[index],'.')
    # plt.legend(['Data cur'], loc='best')
    # plt.grid()



    # plt.show()
    rospy.sleep(1) 

    while j < len(datax) - horizon :

        msg_Waypoints = Waypoints()

        for i in range(0,horizon): 
            pose = Pose2D()

            pose.x = (datax[j+i*rate_in_msg,0])

            pose.y = (datay[j+i*rate_in_msg,0])

            pose.theta = (datath[j+i*rate_in_msg,0])

            msg_Waypoints.velocityList.append((datav_raw[j+i*rate_in_msg,0]))

            msg_Waypoints.curvatureList.append((datacur[j+i*rate_in_msg,0]))

            msg_Waypoints.waypointList.append(pose)


        j += 1
        pub_plan.publish(msg_Waypoints)
        rate.sleep()    

    print('planning ended')
    quit()

def wrap(angle):
    if angle < -np.pi:
        w_angle = 2 * np.pi + angle
    elif angle > np.pi:
        w_angle = angle - 2 * np.pi
    else:
        w_angle = angle

    return w_angle

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
