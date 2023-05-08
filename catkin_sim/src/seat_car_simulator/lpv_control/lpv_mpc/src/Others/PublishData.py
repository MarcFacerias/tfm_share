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

sys.path.append(sys.path[0]+'/ControllerObject')
sys.path.append(sys.path[0]+'/Utilities')

from l4planning_msgs.msg import Waypoints
from geometry_msgs.msg import Pose2D
from utilities import wrap

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
day         = '31_10_2019'
num_test    = 'Test1'
newpath     = '/home/marc/Escritorio/results_simu_test/'+day+'/'+num_test

	
def main():

    rospy.init_node("publishWaypoints")
    rate = rospy.Rate(1)

    with open(newpath+'/datax.txt', "r") as fp: datax_raw = fp.readlines()

    with open(newpath+'/datay.txt', "r") as fp: datay_raw = fp.readlines()

    with open(newpath+'/datath.txt', "r") as fp: datath_raw = fp.readlines()
    
    with open(newpath+'/datav.txt', "r") as fp: datav_raw = fp.readlines()

    with open(newpath+'/datacur.txt', "r") as fp: datacur_raw = fp.readlines()

    pub_plan = rospy.Publisher("waypoints", Waypoints, queue_size=1)

    j = 30
    hist_x = []
    hist_y = []
    hist_v = []
    hist_cur = []
    hist_yaw = []
    flag = True

    while j < 65:#len(datax_raw):

        msg_Waypoints = Waypoints()

        msg_Waypoints.waypointList = []
        msg_Waypoints.velocityList = []
        msg_Waypoints.curvatureList = []

        datax = datax_raw[j].replace('[','')
        datax = datax.replace(']','')
        datax = datax.replace('\n','')
        datax = datax.split(',')

        datay = datay_raw[j].replace('[','')
        datay = datay.replace(']','')
        datay = datay.replace('\n','')
        datay = datay.split(',')

        datath = datath_raw[j].replace('[','')
        datath = datath.replace(']','')
        datath = datath.replace('\n','')
        datath = datath.split(',')

        datav = datav_raw [j].replace('[','')
        datav = datav.replace(']','')
        datav = datav.replace('\n','')
        datav = datav.split(',')

        datacur = datacur_raw [j].replace('[','')
        datacur = datacur.replace(']','')
        datacur = datacur.replace('\n','')
        datacur = datacur.split(',')

        for i in range(1,len(datax)-11): 

            pose = Pose2D()

            pose.x = float(datax[i])

            pose.y = float(datay[i])

            aux = -(float(datath[i]))

            if aux < 0:
                aux += 2*np.pi              

            datath[i] = aux

            # if datath[i] > 2*np.pi:
            #     datath[i] -= 2*np.pi
                
            # if datath[i] < 0:              
            #     datath[i] += 2*np.pi

            pose.theta = (datath[i])

            msg_Waypoints.velocityList.append(float(datav[i]))

            msg_Waypoints.curvatureList.append(float(datacur[i]))

            msg_Waypoints.waypointList.append(pose)

        # on for debug 
        if flag:
            flag = False
            ini_x = float(datax[0])
            ini_y = float(datay[0])
            ini_yaw = float(datath[0])


        hist_x.append(float(datax[0]))
        hist_y.append(float(datay[0]))
        hist_v.append(float(datav[0]))
        hist_cur.append(float(datacur[0]))
        hist_yaw.append(float(datath[0]))

        j = j+1
        pub_plan.publish(msg_Waypoints)
        rate.sleep()    

    t  = range(0,33)
    plt.figure(2)
    plt.subplot(611)
    plt.plot(hist_v, '-')
    plt.legend(['vel'], loc='best')
    plt.grid()

    plt.subplot(612)
    plt.plot(hist_cur, '-')
    plt.legend(['cur'], loc='best')
    plt.grid()

    plt.subplot(613)
    plt.plot(hist_x, '-')
    plt.legend(['X'], loc='best') #Hist_pos[:,0],Hist_pos[:,1],Hist_traj[:,0],Hist_traj[:,1]
    plt.grid()

    plt.subplot(614)
    plt.plot(hist_y, '-')
    plt.legend(['Y'], loc='best')
    plt.grid()

    plt.subplot(615)
    plt.plot((hist_x),(hist_y), '-')
    plt.legend(['X vs Y'], loc='best')
    plt.grid()

    plt.subplot(616)
    plt.plot(hist_yaw, '-')
    plt.legend(['yaw'], loc='best')
    plt.grid()

    plt.show()

    print('planning ended')
    quit()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
