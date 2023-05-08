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

from utilities import Regression, Curvature
from dataStructures import LMPCprediction, EstimatorData, PlanningData, ClosedLoopDataObj #planning data-> sub a "my points"
from PathFollowingLPVMPC import PathFollowingLPV_MPC, ABC_computation_5SV
from l4vehicle_msgs.msg import ModelPredictiveControlCommand, VehicleState
from l4planning_msgs.msg import Waypoints
from geometry_msgs.msg import Pose2D
from scipy.io import loadmat
from scipy.interpolate import interp1d
from numpy import tan, arctan, cos, sin, pi


np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
dt          = 1
day         = '31_10_2019'
num_test    = 'Test1'
newpath     = '/home/marc/Escritorio/results_simu_test/'+day+'/'+num_test

def main():

    with open(newpath+'/datav.txt', "r") as fp: datav_raw_s = fp.readlines()

    with open(newpath+'/datacur.txt', "r") as fp: datacur_raw_s = fp.readlines()

    with open(newpath+'/datax.txt', "r") as fp: datax_raw_s = fp.readlines()

    with open(newpath+'/datay.txt', "r") as fp: datay_raw_s = fp.readlines()

    with open(newpath+'/datath.txt', "r") as fp: datath_raw_s = fp.readlines()

    rospy.init_node("testRefs")
    N = len(datav_raw_s)
    computed_datath    = np.zeros(N)
    computed_datax     = np.zeros(N)
    computed_datay     = np.zeros(N)
    computed_datavx    = np.zeros(N)
    computed_datavy    = np.zeros(N)
    computed_datav     = np.zeros(N)
    computed_dataW     = np.zeros(N)
    computed_datacur   = np.zeros(N)

    datav_raw          = np.zeros(N)
    datacur_raw        = np.zeros(N)
    datax_raw          = np.zeros(N)
    datay_raw          = np.zeros(N)
    datath_raw         = np.zeros(N)
    
    print(len(datav_raw_s))

    for j in range (0,len(datav_raw_s)-1):
        datav = datav_raw_s [j].replace('[','')
        datav = datav.replace(']','')
        datav = datav.replace('\n','')
        datav = datav.split(',')
        # datav_raw[j*10:j*10+10] = datav[0:10]
        datav_raw[j] = datav[0]

        datacur = datacur_raw_s[j].replace('[','')
        datacur = datacur.replace(']','')
        datacur = datacur.replace('\n','')
        datacur = datacur.split(',')
        # datacur_raw[j*10:j*10+10] = datacur[0:10] 
        datacur_raw[j] = datacur[0]

        datax = datax_raw_s[j].replace('[','')
        datax = datax.replace(']','')
        datax = datax.replace('\n','')
        datax = datax.split(',')
        # datax_raw[j*10:j*10+10] = datax[0:10] 
        datax_raw[j] = datax[0]

        datay = datay_raw_s[j].replace('[','')
        datay = datay.replace(']','')
        datay = datay.replace('\n','')
        datay = datay.split(',')
        # datay_raw[j*10:j*10+10] = datay[0:10] 
        datay_raw[j] = datay[0]

        datath = datath_raw_s[j].replace('[','')
        datath = datath.replace(']','')
        datath = datath.replace('\n','')
        datath = datath.split(',')
        # datath_raw[j*10:j*10+10] = datath[0:10] 
        datath_raw[j] = datath[0]
        if datath_raw[j] > pi:
            datath_raw[j] = datath_raw[j] - 2*pi

    for i in range(0,len(datacur_raw)-1): 
        computed_datav[i]     = float(datav_raw[i])
        computed_datacur[i]   = float(datacur_raw[i])
        computed_dataW[i]     = computed_datav[i]  * computed_datacur[i]

    computed_datath[0] = float(datath_raw[0]) 
    computed_datax[0]  = float(datax_raw[0])
    computed_datay[0]  = float(datay_raw[0])

    for i in range(1,len(datacur_raw)-1):

        computed_datath[i] = computed_datath[i-1] + computed_dataW[i]*dt
        computed_datavx[i] = computed_datav[i] * cos(computed_datath[i])
        computed_datavy[i] = computed_datav[i] * sin(computed_datath[i])

        computed_datax[i]  = computed_datax[i-1] + dt*computed_datavx[i]
        computed_datay[i]  = computed_datay[i-1] + dt*computed_datavy[i]


    # Interpolation

    index = np.arange(0,len(computed_datav),1)
    mod_vel = interp1d(index,computed_datav)
    mod_x   = interp1d(index,computed_datax)
    mod_y   = interp1d(index,computed_datay)
    mod_yaw = interp1d(index,computed_datath)
    mod_w   = interp1d(index,computed_dataW)
    mod_cur = interp1d(index,computed_datacur)
    indexRS = np.arange(0,len(computed_datav)-0.9,0.1)

    ups_vel = mod_vel(indexRS)
    ups_x   = mod_x(indexRS)   
    ups_y   = mod_y (indexRS)
    ups_yaw = mod_yaw(indexRS)
    ups_w   = mod_w(indexRS)
    ups_cur = mod_cur(indexRS)

    #print(ups_vel.reshape((-1,1)))

    Refs = np.concatenate((ups_vel.reshape((-1,1)), ups_w.reshape((-1,1)), 
           ups_cur.reshape((-1,1)),ups_x.reshape((-1,1)), ups_y.reshape((-1,1)), ups_yaw.reshape((-1,1))),axis = 1)
    np.save('newRefs',Refs)

    #TODO: subplot con las origales
    plt.figure();
    plt.subplot(611)
    plt.plot(indexRS,ups_y)
    plt.legend(['Track Y'], loc='best')
    plt.grid()

    plt.subplot(612)
    plt.plot(indexRS,ups_x)
    plt.legend(['Track X'], loc='best')
    plt.grid()

    plt.subplot(613)
    plt.plot(indexRS,ups_vel)
    # plt.plot(index,computed_datav[index],'-')
    # plt.plot(indexRS,new_path[indexRS],'r.')
    plt.legend(['Vel x'], loc='best')
    plt.grid()

    plt.subplot(614)
    plt.plot(indexRS,ups_w)
    plt.legend(['vel ang'], loc='best')
    plt.grid()

    plt.subplot(615)
    plt.plot(indexRS,ups_yaw)
    plt.legend(['Yaw'], loc='best')
    plt.grid()

    plt.subplot(616)
    plt.plot(indexRS,ups_cur)
    plt.legend(['Data cur'], loc='best')
    plt.grid()

    plt.show(block = False)
    rospy.sleep(1)

        #TODO: subplot con las origales
    plt.figure();
    plt.subplot(611)
    plt.plot(datay_raw)
    plt.legend(['Original Track Y'], loc='best')
    plt.grid()

    plt.subplot(612)
    plt.plot(datax_raw)
    plt.legend(['Original Track X'], loc='best')
    plt.grid()

    plt.subplot(613)
    plt.plot(datav_raw)
    plt.legend(['Original Vel x'], loc='best')
    plt.grid()

    plt.subplot(614)
    plt.plot(computed_dataW)
    plt.legend(['Original Vel ang'], loc='best')
    plt.grid()

    plt.subplot(615)
    plt.plot(datath_raw)
    plt.legend(['Original Yaw'], loc='best')
    plt.grid()

    plt.subplot(616)
    plt.plot(datacur_raw)
    plt.legend(['Original Data cur'], loc='best')
    plt.grid()

    plt.show()
    rospy.sleep(1)
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
