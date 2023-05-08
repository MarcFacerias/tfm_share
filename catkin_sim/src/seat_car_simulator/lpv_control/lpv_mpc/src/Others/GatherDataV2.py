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

import rospy
import geometry_msgs.msg
from lpv_mpc.msg import ECU, pos_info, Vel_est, simulatorStates, My_IMU
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from marvelmind_nav.msg import hedge_imu_fusion, hedge_pos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from l4vehicle_msgs.msg import VehicleState
from numpy import tan, arctan, cos, sin, pi
from numpy.random import randn,rand
from tf import transformations
import numpy as np
import pdb

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
num_test    = 'RosBag3'
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

class FlagCar(object):

    def __init__(self):

        rospy.Subscriber('flag', Bool, self.flag_callback, queue_size=1)
        self.flag = 0

    def flag_callback(self,msg):

        self.flag = msg.data

class FlagSim(object):

    def __init__(self):

        rospy.Subscriber('SyncFlag', Bool, self.flag_callback, queue_size=1)
        self.flag = 0

    def flag_callback(self,msg):

        self.flag = msg.data

class StatesCar(object):

    def __init__(self):

		rospy.Subscriber('vehicle_state', VehicleState, self.vehicle_callback, queue_size=1)
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
    	rospy.Subscriber('sensorStates',simulatorStates, self.simulatorStates_callback, queue_size=1)
    	self.simulatorData = np.zeros([1,6])

class controlActions(object):
    def controlActions_callback(self,msg):
        self.controlActions = [msg.motor, msg.servo]
        f = open(newpath+'/controlActions.txt','a')
        f.write(str(self.controlActions)+'\n')
        f.close()

    def __init__(self):
        rospy.Subscriber('ecu', ECU, self.controlActions_callback, queue_size=1)
        self.controlActions = np.zeros([1,2])

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////   

class StatesCar_s(object):

    def __init__(self):

        rospy.Subscriber('vehicle_state_s', VehicleState, self.vehicle_callback, queue_size=1)
        self.vehicleData = np.zeros([1,6])

    def vehicle_callback(self,msg):

        self.vehicleData = [msg.longitudinal_velocity, msg.lateral_velocity, msg.angular_velocity, msg.x, msg.y, msg.heading] 
        f = open(newpath+'/vehicleData_s.txt','a')
        f.write(str(self.vehicleData)+'\n')
        f.close()


class StatesSim_s(object):
    def simulatorStates_callback(self,msg):
        self.simulatorData = [msg.vx, msg.vy, msg.psiDot, msg.x, msg.y, msg.psi]
        f = open(newpath+'/sensorStates_s.txt','a')
        f.write(str(self.simulatorData)+'\n')
        f.close()

    def __init__(self):
        rospy.Subscriber('sensorStates_s', simulatorStates, self.simulatorStates_callback, queue_size=1)
        self.simulatorData = np.zeros([1,6])

class controlActions_s(object):
    def controlActions_callback(self,msg):
        self.controlActions = [msg.motor, msg.servo]
        f = open(newpath+'/controlActions_s.txt','a')
        f.write(str(self.controlActions)+'\n')
        f.close()

    def __init__(self):
        rospy.Subscriber('ecu_s', ECU, self.controlActions_callback, queue_size=1)
        self.controlActions = np.zeros([1,2])     

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class StatesEst(object):
    def StatesEst_callback(self,msg):
    	self.estimadtorData = [msg.s, msg.ey, msg.epsi, msg.v, msg.x, msg.y, msg.v_x, msg.v_y, msg.psi, msg.psiDot, msg.a_x, msg.a_y, msg.u_a, msg.u_df]
    	f = open(newpath+'/StEstim.txt','a')
    	f.write(str(self.estimadtorData)+'\n')
    	f.close()

    def __init__(self):
    	rospy.Subscriber('pos_info', pos_info, self.StatesEst_callback, queue_size=1)
    	self.estimadtorData = np.zeros([1,15])


	

def main():

    rospy.init_node("recordWaypoints")

    Flag1 = 0
    Flag2 = 0
    init2 = FlagCar()
    init1 = FlagSim()

    while (not rospy.is_shutdown()):

    	if init2.flag==1 and Flag1==0:
            init3 = StatesCar()
            init4 = StatesSim()
            init6 = controlActions()
            Flag1 = True

        if init1.flag==1 and Flag2==0:
            init7 = StatesCar_s()
            init8 = StatesSim_s()
            init0 = controlActions_s()
            Flag2 = True

        pass
    quit()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
