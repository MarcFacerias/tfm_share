#!/usr/bin/env python
"""
    File name: barc_simulator_dyn.py
    Author: Shuqi Xu
    Email: shuqixu@kth.se
    Python Version: 2.7.12
"""
# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
#----------------------------------------------------------------------------


from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Bool
import sys
import os
sys.path.append(sys.path[0]+'/ControllersObject')
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

sim_lnd = True

def main():
    rospy.init_node("simulator")
    gps_freq_update = rospy.get_param("simulator/gps_freq_update")
    simulator_dt    = rospy.get_param("simulator/dt")
    lowLevelDyn     = rospy.get_param("simulator/lowLevelDyn")
    # pub_rate   = rospy.get_param("simulator/pub_rate")

    print ("[SIMULATOR] Is Low Level Dynamics active?: ", lowLevelDyn)

    sim = Simulator()
    gps = GpsClass(gps_freq_update, simulator_dt)
    ecu = EcuClass()
    link_msg = ModelState()
    flag  = FlagClass()
    hist_sens_noise = []

    link_msg.model_name = "seat_car"
    link_msg.reference_frame = "world"

    link_msg.pose.position.z = 0.031
    link_msg.twist.linear.x = 0.0
    link_msg.twist.linear.y = 0
    link_msg.twist.linear.z = 0

    link_msg.twist.angular.x = 0.0
    link_msg.twist.angular.y = 0
    link_msg.twist.angular.z = 0

    a_his 	= [0.0]*int(rospy.get_param("simulator/delay_a")/rospy.get_param("simulator/dt"))
    df_his 	= [0.0]*int(rospy.get_param("simulator/delay_df")/rospy.get_param("simulator/dt"))

    sim_offset_x 	= rospy.get_param("simulator/init_x")
    sim_offset_y 	= rospy.get_param("simulator/init_y")
    sim_offset_yaw 	= rospy.get_param("simulator/init_yaw")

    pub_simulatorStates = rospy.Publisher('simulatorStates', simulatorStates, queue_size=1)
    if sim_lnd:
    	land_pub             = rospy.Publisher("/Corrected_Pose_s",Bool , queue_size=1)
    pub_simulatorStatesIDIADA = rospy.Publisher('vehicle_state', VehicleState, queue_size=1)
    pub_linkStates = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pub_sysOut = rospy.Publisher('sensorStates', simulatorStates, queue_size=1)



    simStatesIDADA = VehicleState()
    simStates = simulatorStates()

    print('[SIMULATOR] The simulator is running!')
    print('\n')

    servo_inp = 0.0
    T  = simulator_dt
    Tf = 0.07
    counter = 0

    while not (rospy.is_shutdown()):

		#callback_time = rospy.get_time()

		if flag.status:
			pass
    	# Simulator delayc
		counter += 1
		a_his.append(ecu.u[0])
		df_his.append(ecu.u[1])

		if lowLevelDyn == True:
			servo_inp = (1 - T / Tf) * servo_inp + ( T / Tf )*df_his.pop(0)
			u = [a_his.pop(0), servo_inp]
		else:
			u = [a_his.pop(0), df_his.pop(0)] # EA: remove the first element of the array and return it to you.
			#print "Applyied the first of this vector: ",df_his ,"\n "

		sim.f(u)

		simStates.x      = sim.x
		simStates.y      = sim.y
		simStates.vx     = sim.vx
		simStates.vy     = sim.vy
		simStates.psi    = sim.yaw
		simStates.psiDot = sim.psiDot

		simStatesIDADA.x                         = sim.x
		simStatesIDADA.y                         = sim.y
		simStatesIDADA.longitudinal_velocity     = sim.vx
		simStatesIDADA.lateral_velocity          = sim.vy
		simStatesIDADA.heading                   = sim.yaw
		simStatesIDADA.angular_velocity          = sim.psiDot

		#if counter >= pub_rate/simulator_dt:
			# Publish input

		pub_simulatorStates.publish(simStates)
		pub_simulatorStatesIDIADA.publish(simStatesIDADA)
		aux = SimulateSensors(sim, pub_sysOut)
		hist_sens_noise.append(aux)

		counter = 0

		gps.update(sim)

		sim.saveHistory()

		gps.gps_pub()

		if pub_linkStates.get_num_connections() > 0:
			link_msg.pose.position.x = sim.x + sim_offset_x
			link_msg.pose.position.y = -(sim.y + sim_offset_y)
			link_msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, -sim.yaw))
			pub_linkStates.publish(link_msg)
		if sim_lnd:
			mrk = Bool()
			mrk.data = sim_lnd
			land_pub.publish(mrk)

		# # ESTO SIN COMENTAR ES COMO ESTABA ANTES...
		# counter += 1
		# if counter == 5:
		# 	enc.enc_pub()
		# 	counter = 0


		#time_since_last_callback = rospy.get_time() - callback_time

		#print time_since_last_callback
		sim.rate.sleep()

    str         = 'test2'
    day         = 'TestPreFinal'
    num_test    = 'Noise'
    newpath     = '/home/marc/Escritorio/results_simu_test/'+day+'/'+num_test+'/'
    print ("Hi!")
    print (len(hist_sens_noise))
    print (newpath+'NoiseSens')
    if not os.path.exists(newpath+'/'):
        os.makedirs(newpath+'/')
    np.save(newpath+'NoiseSens', hist_sens_noise)
    np.save(newpath+'NoiseProces', sim.noise_hist)
    np.save(newpath+'NoiseU', ecu.hist_noise)
    quit()


def SimulateSensors(simu, pub):
	msg = simulatorStates()

	n1 = max(-simu.vx_std_s * simu.n_bound_s, min(simu.vx_std_s * (4*rand()-2), simu.vx_std_s * simu.n_bound_s))
	n2 = max(-simu.psiDot_std_s * simu.n_bound_s, min(simu.psiDot_std_s * (4*rand()-2), simu.psiDot_std_s * simu.n_bound_s))
	n3 = max(-simu.x_std_s * simu.n_bound_s, min(simu.x_std_s * (4*rand()-2), simu.x_std_s * simu.n_bound_s))
	n4 = max(-simu.y_std_s * simu.n_bound_s, min(simu.y_std_s * (4*rand()-2), simu.y_std_s * simu.n_bound_s))
	n5 = max(-simu.psi_std_s * simu.n_bound_s, min(simu.psi_std_s * (4*rand()-2), simu.psi_std_s * simu.n_bound_s))


	if simu.vx > 0.75:

		msg.vx = simu.vx + n1

		msg.vy = 0#simu.vy + n

		msg.psiDot = simu.psiDot + n2

		msg.x = simu.x + n3

		msg.y = simu.y + n4

		msg.psi = simu.yaw + n5

	else:

		msg.vx = simu.vx
		msg.vy = simu.vy
		msg.psiDot = simu.psiDot
		msg.x = simu.x
		msg.y = simu.y
		msg.psi = simu.yaw

	pub.publish(msg)
	return [n1,n2,n3,n4,n5]





class Simulator(object):
	""" Object collecting GPS measurement data
    Attributes:
    	Model params:
    		1.L_f 2.L_r 3.m(car mass) 3.I_z(car inertial) 4.c_f(equivalent drag coefficient)
        States:
            1.x 2.y 3.vx 4.vy 5.ax 6.ay 7.psiDot
        States history:
            1.x_his 2.y_his 3.vx_his 4.vy_his 5.ax_his 6.ay_his 7.psiDot_his
        Simulator sampling time:
        	1. dt
        Time stamp:
        	1. time_his
	Methods:
		f(u):
			System model used to update the states
		pacejka(ang):
			Pacejka lateral tire modeling
    """
	def __init__(self):

		self.L_f 	= rospy.get_param("lf")
		self.L_r 	= rospy.get_param("lr")
		self.m 		= rospy.get_param("m")
		self.I_z 	= rospy.get_param("Iz")
		self.Cf 	= rospy.get_param("Cf")
		self.Cr 	= rospy.get_param("Cr")
		self.mu 	= rospy.get_param("mu")

		self.g = 9.81

		# with noise
		self.x 		= 0.0
		self.y 		= 0.0
		self.vx 	= 0.1#rospy.get_param("init_vx")
		self.vy 	= 0.0
		self.ax 	= 0.0
		self.ay 	= 0.0

		self.yaw 	    = rospy.get_param("simulator/init_yaw")
		self.dist_mode 	= rospy.get_param("simulator/dist_mode")
		self.mu_sf 	    = rospy.get_param("simulator/mu_sf")
		self.Cd         = rospy.get_param("simulator/Cd")
		self.A_car      = rospy.get_param("simulator/A_car")


		self.psiDot = 0.0

		self.x_his 		= []
		self.y_his 		= []
		self.vx_his 	= []
		self.vy_his 	= []
		self.ax_his 	= []
		self.ay_his 	= []
		self.psiDot_his = []
		self.noise_hist = []

		self.dt 		= rospy.get_param("simulator/dt")
		self.rate 		= rospy.Rate(1.0/self.dt)
		# self.rate 		= rospy.Rate(1.0)
		self.time_his 	= []

        # Get process noise limits
		self.x_std           = rospy.get_param("simulator/x_std_pr")/self.dt
		self.y_std           = rospy.get_param("simulator/y_std_pr")/self.dt
		self.vx_std          = rospy.get_param("simulator/vx_std_pr")/self.dt
		self.vy_std          = rospy.get_param("simulator/vy_std_pr")/self.dt
		self.psiDot_std      = rospy.get_param("simulator/psiDot_std_pr")/self.dt
		self.psi_std         = rospy.get_param("simulator/psi_std_pr")/self.dt
		self.n_bound 	     = rospy.get_param("simulator/n_bound_pr")/self.dt

		#Get sensor noise limits

		self.x_std_s           = rospy.get_param("simulator/x_std")
		self.y_std_s           = rospy.get_param("simulator/y_std")
		self.vx_std_s          = rospy.get_param("simulator/vx_std")
		self.vy_std_s          = rospy.get_param("simulator/vy_std")
		self.psiDot_std_s      = rospy.get_param("simulator/psiDot_std")
		self.psi_std_s         = rospy.get_param("simulator/psi_std")
		self.n_bound_s         = rospy.get_param("simulator/n_bound")

	def f(self,u):
		a_F = 0.0
		a_R = 0.0

		if abs(self.vx) > 0.7:
			a_F = u[1] - arctan((self.vy + self.L_f*self.psiDot)/abs(self.vx))
			a_R = arctan((- self.vy + self.L_r*self.psiDot)/abs(self.vx))

		# FyF = self.pacejka(a_F)
		# FyR = self.pacejka(a_R)

		FyF = self.Cf * a_F
		FyR = self.Cr * a_R


		if abs(a_F) > 30.0/180.0*pi or abs(a_R) > 30.0/180.0*pi:
			print ("WARNING: Large slip angles in simulation")

		x 	    = self.x
		y 	    = self.y
		ax 	    = self.ax
		ay   	= self.ay
		vx 	    = self.vx
		vy 	    = self.vy
		yaw     = self.yaw
		psiDot  = self.psiDot

		if self.dist_mode:

			dist = ( 10*self.Cd*1.225*self.A_car*(vx**2) + self.mu_sf*9.81*self.m)/self.m

		else:
			dist = self.mu*vx

		#print("disturbance: " , dist)


		#despreciem forces longitudinals i no fem l'aproximacio rara dels angles (respecte al pdf)

		n4 = max(-self.x_std*self.n_bound, min(self.x_std*0.66*(randn()), self.x_std*self.n_bound))
		self.x 		+= self.dt*(cos(yaw)*vx - sin(yaw)*vy + n4)

		n5 = max(-self.y_std*self.n_bound, min(self.y_std*0.66*(randn()), self.y_std*self.n_bound))
		self.y 		+= self.dt*(sin(yaw)*vx + cos(yaw)*vy + n5)

		n1 = max(-self.vx_std*self.n_bound, min(self.vx_std*0.66*(randn()), self.vx_std*self.n_bound))
		self.vx 	+= self.dt*(ax + psiDot*vy + n1 - dist)

		n2 = max(-self.vy_std*self.n_bound, min(self.vy_std*0.66*(randn()), self.vy_std*self.n_bound))
		self.vy 	+= self.dt*(ay - psiDot*vx + n2)

		# self.ax 	 = u[0]*cos(u[1]) - self.mu*vx - FyF/self.m*sin(u[1])  # front driven vehicle
		# self.ay 	 = u[0]*sin(u[1]) + 1.0/self.m*(FyF*cos(u[1])+FyR)
		self.ax 	 = u[0] - FyF/self.m*sin(u[1])  # front driven vehicle
		self.ay 	 = 1.0/self.m*(FyF*cos(u[1])+FyR)

		#n3 = 0.66*(randn());
		n3 = max(-self.psi_std*self.n_bound, min(self.psi_std*0.66*(randn()), self.psi_std*self.n_bound))
		self.yaw 	+= self.dt*(psiDot + n3)

		n6 = max(-self.psiDot_std*self.n_bound, min(self.psiDot_std*0.66*(randn()), self.psiDot_std*self.n_bound))
		self.psiDot     += self.dt*((self.L_f*FyF*cos(u[1]) - self.L_r*FyR)/self.I_z + n6)

		self.vx = abs(self.vx)
		self.noise_hist.append([n1,n2,n6,n4,n5,n3])


	def pacejka(self,ang):
		D = self.c_f*self.m*self.g/2 	# Friction force/2
		Stiffness_Force = D*sin(self.C*arctan(self.B*ang))
		return Stiffness_Force

	def saveHistory(self):
		self.x_his.append(self.x)
		self.y_his.append(self.y)
		self.vx_his.append(self.vx)
		self.vy_his.append(self.vy)
		self.ax_his.append(self.ax)
		self.ay_his.append(self.ay)
		self.psiDot_his.append(self.psiDot)
		self.time_his.append(rospy.get_rostime().to_sec())

class GpsClass(object):
	def __init__(self, gps_freq_update, simulator_dt):
		self.pub  = rospy.Publisher("hedge_pos", hedge_pos, queue_size=1)
		self.x = 0.0
		self.y = 0.0
		self.x_std 	 = rospy.get_param("simulator/x_std")
		self.y_std 	 = rospy.get_param("simulator/y_std")
		self.n_bound = rospy.get_param("simulator/n_bound")

		self.msg = hedge_pos()
		self.counter  = 0
		self.thUpdate = (1.0 /  gps_freq_update) / simulator_dt

	def update(self,sim):
		n = max(-self.x_std*self.n_bound, min(self.x_std*randn(), self.x_std*self.n_bound))
		self.x = sim.x + n

		n = max(-self.y_std*self.n_bound, min(self.y_std*randn(), self.y_std*self.n_bound))
		self.y = sim.y + n

	def gps_pub(self):
		if self.counter > self.thUpdate:
			self.counter = 0
			self.msg.x_m = self.x
			self.msg.y_m = self.y
			self.pub.publish(self.msg)
			# print "Update GPS"
		else:
			# print "Not update GPS"
			self.counter = self.counter + 1

class EcuClass(object):
	def __init__(self):
		self.sub = rospy.Subscriber("ecu", ECU, self.ecu_callback, queue_size=1)
		self.u = [0.0, 0.0]
		self.du_0 	 = rospy.get_param("simulator/du_0")
		self.du_1 	 = rospy.get_param("simulator/du_1")
		self.u_bound = rospy.get_param("simulator/u_bound")
		self.hist_noise = []

	def ecu_callback(self,data):

		n1 = max(-self.du_0*self.u_bound, min(self.du_0*(2*rand()-1), self.du_0*self.u_bound))
		n2 = max(-self.du_1*self.u_bound, min(self.du_1*(2*rand()-1), self.du_1*self.u_bound))
		self.hist_noise.append([n1,n2])
		self.u = [data.motor + n1, data.servo + n2]


class FlagClass(object):

	def __init__(self):
		self.flag_sub = rospy.Subscriber("flag", Bool, self.flag_callback, queue_size=1)
		self.status   = False
	def flag_callback(self,msg):
		self.status = msg.data

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
		pass
