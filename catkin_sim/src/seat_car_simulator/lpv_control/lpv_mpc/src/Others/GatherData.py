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

# from trackInitialization import Map, wrap
# from barc.msg import ECU, prediction, Racing_Info, My_Planning, simulatorStates, pos_info
# from utilities import Regression, Curvature
# from dataStructures import LMPCprediction, EstimatorData, PlanningData, ClosedLoopDataObj #planning data-> sub a "my points"
# from PathFollowingLPVMPC import PathFollowingLPV_MPC, ABC_computation_5SV
# from l4vehicle_msgs.msg import ModelPredictiveControlCommand, VehicleState
from l4planning_msgs.msg import Waypoints

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
num_test    = 'RosBag2'
newpath     = '/home/marc/Escritorio/results_simu_test/'+num_test

class PlanningData(object):
    """ Object collecting data from planning node """

    def __init__(self):

        self.x_d = []
        self.y_d = []
        self.psi_d = []
        self.vx_d = []
        self.curv_d = []
        self.vel_ant = 0
        self.counter = 0
        self.continuity = 0
        self.y_ant = 0
        self.x_ant = 0
        self.msg_waypoints = []               
        # rospy.Subscriber('My_Planning', My_Planning, self.My_Planning_callback, queue_size=1)
        rospy.Subscriber('inertial_waypoints', Waypoints, self.Waypoints_callback, queue_size=1)

    def Waypoints_callback(self,msg):
        """ ... """
        
        if not(len( msg.waypointList ) == 0) and msg.waypointList[0].x != self.x_ant and msg.waypointList[0].y != self.y_ant:

            for pose in msg.waypointList:
                self.x_d.append(pose.x)       
                self.y_d.append(pose.y)
                self.psi_d.append(pose.theta * 3.1416/180)
      
            for vel in msg.velocityList:
                self.vx_d.append(vel)

            for curv in msg.curvatureList:
                self.curv_d.append(curv)

            x_ant = msg.waypointList[0].x
            y_ant = msg.waypointList[0].y

            print('cur[0] ', msg.curvatureList[0], 'vx[0]', msg.velocityList[0],'cur[end]', msg.curvatureList[-1], 'vx[end]', msg.velocityList[-1], 'length plan', len(msg.velocityList))
            print('\n')
            print('x[0] ', msg.waypointList[0].x, 'y[0]', msg.waypointList[0].y,'x[end]', msg.waypointList[-1].x, 'y[end]', msg.waypointList[-1].x)
            print('\n')
            if self.vel_ant == msg.velocityList[0]:
                self.counter += 1
            else:
                print('different msg after',self.counter, 'vel_act' , msg.velocityList[0], 'continuity check', self.continuity)
                self.counter = 0
            self.vel_ant = msg.velocityList[0]

            if self.counter*10 < len(msg.velocityList):
                self.continuity = msg.velocityList[(self.counter*10)]
            else: 
                self.continuity = msg.velocityList[-1]

            f = open(newpath+'/datax.txt','a')
            f.write(str(self.x_d)+'\n')
            f.close()

            f = open(newpath+'/datay.txt','a')
            f.write(str(self.y_d)+'\n')
            f.close()

            f = open(newpath+'/datath.txt','a')
            f.write(str(self.psi_d)+'\n')
            f.close()

            f = open(newpath+'/datav.txt','a')
            f.write(str(self.vx_d)+'\n')
            f.close()

            f = open(newpath+'/datacur.txt','a')
            f.write(str(self.curv_d)+'\n')
            f.close()

            self.x_d = []
            self.y_d = []
            self.psi_d = []
            self.vx_d = []
            self.curv_d = []

        else:
        	pass
            

class StatesCar(object):

    def __init__(self):

		rospy.Subscriber('VehicleState', VehicleState, self.vehicle_callback, queue_size=1)
		self.vehicleData = np.zeros([1,6])

    def vehicle_callback(self,msg):

    	self.vehicleData = [msg.longitudinal_velocity, msg.lateral_velocity, msg.angular_velocity, msg.x, msg.y, msg.heading] 
    	f = open(newpath+'/vehicleData.txt','a')
    	f.write(str(self.vehicleData)+'\n')
    	f.close()


class StatesSim(object):
    def simulatorStates_callback(self,msg):
    	self.simulatorData = [msg.vx, msg.vy, msg.psiDot, msg.x, msg.y, msg.psi]
    	f = open(newpath+'/VehicleState.txt','a')
    	f.write(str(self.simulatorData)+'\n')
    	f.close()

    def __init__(self):
    	rospy.Subscriber('simulatorStates', simulatorStates, self.simulatorStates_callback, queue_size=1)
    	self.simulatorData = np.zeros([1,6])

class StatesEst(object):
    def StatesEst_callback(self,msg):
    	self.estimadtorData = [msg.s, msg.ey, msg.epsi, msg.v, msg.x, msg.y, msg.v_x, msg.v_y, msg.psi, msg.psiDot, msg.a_x, msg.a_y, msg.u_a, msg.u_df]
    	f = open(newpath+'/StEstim.txt','a')
    	f.write(str(self.estimadtorData)+'\n')
    	f.close()

    def __init__(self):
    	rospy.Subscriber('pos_info', pos_info, self.StatesEst_callback, queue_size=1)
    	self.estimadtorData = np.zeros([1,15])

class controlActions(object):
    def controlActions_callback(self,msg):
    	self.controlActions = [msg.motor, msg.servo]
    	f = open(newpath+'/controlActions.txt','a')
    	f.write(str(self.controlActions)+'\n')
    	f.close()

    def __init__(self):
    	rospy.Subscriber('ecu', ECU, self.controlActions_callback, queue_size=1)
    	self.controlActions = np.zeros([1,2])    	
	

def main():

    rospy.init_node("recordWaypoints")

    f = open(newpath+'/datax.txt','w')
    f.close()
    f = open(newpath+'/datay.txt','w')
    f.close()
    f = open(newpath+'/datath.txt','w')
    f.close()
    f = open(newpath+'/datav.txt','w')
    f.close()
    f = open(newpath+'/datacur.txt','w')
    f.close()

    init = PlanningData()
    # Flag = 0
    #init2 = StatesCar()
    # init3 = StatesSim()
    # init4 = StatesEst()
    # init5 = controlActions()

    while (not rospy.is_shutdown()):

    	# if init.end==1 and Flag==0:
		   #  init2 = StatesCar()
		   #  init3 = StatesSim()
		   #  init4 = StatesEst()
		   #  init5 = controlActions()
		   #  Flag=1;

        pass
    quit()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
