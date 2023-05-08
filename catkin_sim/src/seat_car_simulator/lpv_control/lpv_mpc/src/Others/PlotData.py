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
num_test    = 'RosBag2'
newpath     = '/home/marc/Escritorio/results_simu_test/'+num_test

	
def main():

    rospy.init_node("printhWaypoints")
    rate = rospy.Rate(1)

    with open(newpath+'/datax.txt', "r") as fp: datax_raw = fp.readlines()

    with open(newpath+'/datay.txt', "r") as fp: datay_raw = fp.readlines()

    with open(newpath+'/datath.txt', "r") as fp: datath_raw = fp.readlines()
    
    with open(newpath+'/datav.txt', "r") as fp: datav_raw = fp.readlines()

    with open(newpath+'/datacur.txt', "r") as fp: datacur_raw = fp.readlines()

    pub_plan = rospy.Publisher("waypoints", Waypoints, queue_size=1)

    j = 0
    hist_x = []
    hist_y = []
    hist_v = []
    hist_cur = []
    hist_yaw = []
    hist_w = []
    flag = True

    while j < len(datax_raw):

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

        if flag:
            flag = False
            ini_x = float(datax[0])
            print(ini_x)
            ini_y = float(datay[0])
            print(ini_y)
            ini_yaw = float(datath[0])
            print(ini_yaw)

        

        hist_x.append(float(datax[0]))
        hist_y.append(float(datay[0]))
        hist_v.append(float(datav[0]))
        hist_cur.append(float(datacur[0]))
        hist_yaw.append(float(datath[0]))
        hist_w.append(float(datacur[0])*float(datav[0]))

        temp_x   = datax[:]
        temp_y   = datay[:]
        temp_v   = datav[:]
        temp_cur = datacur[:]
        temp_yaw = datath[:]

        j = j+1


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
    plt.plot(hist_x, '.')
    plt.legend(['X'], loc='best')
    plt.grid()
    
    plt.subplot(614)
    plt.plot(hist_y, '.')
    plt.legend(['Y'], loc='best')
    plt.grid()

    plt.subplot(615)
    plt.plot(hist_yaw, '-')
    plt.legend(['yaw'], loc='best')
    plt.grid()

    plt.subplot(616)
    plt.plot(hist_w, '-')
    plt.legend(['ang vel'], loc='best')
    plt.grid()

    plt.draw()
    plt.show()        
    rate.sleep()           
    plt.clf()
        
    print('planning ended')
    quit()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
