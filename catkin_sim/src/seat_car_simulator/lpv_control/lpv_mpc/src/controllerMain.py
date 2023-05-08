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

from lpv_mpc.msg import ECU, prediction, Racing_Info, My_Planning
from std_msgs.msg import Bool, Int16, UInt8
from l4vehicle_msgs.msg import ModelPredictiveControlCommand
from utilities import Regression, Curvature, wrap
from dataStructures import LMPCprediction, EstimatorData, PlanningData
from PathFollowingLPVMPC import PathFollowingLPV_MPC, ABC_computation_5SV
from trackInitialization import Map, wrap

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


def main():

    rospy.init_node("LPV-MPC")
    input_commands  = rospy.Publisher('ecu', ECU, queue_size=1)
    pub_flag        = rospy.Publisher('flag', Bool, queue_size=1)
    pred_treajecto  = rospy.Publisher('OL_predictions', prediction, queue_size=1)
    OL_predictions  = prediction()

    mode            = rospy.get_param("/control/mode")
    N               = rospy.get_param("/control/N")
    Vx_ref          = rospy.get_param("/control/vel_ref")

    controlVel_commands  = rospy.Publisher('/manual_control/speed',Int16,queue_size=1)
    controlSter_commands = rospy.Publisher('steering',UInt8,queue_size=1)

    loop_rate       = rospy.get_param("/control/Hz")
    dt              = 1.0/loop_rate
    rate            = rospy.Rate(loop_rate)
    rise_flag       = Bool()


    ## TODO: change this in order to be taken from the launch file
    Steering_Delay  = 0 #3
    Velocity_Delay  = 0

    #Steering_Delay  = int(rospy.get_param("/simulator/delay_df")/dt)

    NN_LPV_MPC      = False

    u_steer = 0
    u_acc = 0

#########################################################
#########################################################


    # Objects initializations
    map             = Map()
    cmd             = ECU()                                              # Command message
    cmd.servo       = 0.0
    cmd.motor       = 0.0                    # Closed-Loop Data

    # Var IDIADA
    # cmd_car = ModelPredictiveControlCommand()
    # cmd_car.left_steer_turn_radius_reciprocal_command = 0
    # cmd_car.acceleration_command = 0
    # cmd_car.brake_deceleration_command = 0

    estimatorData   = EstimatorData()                                      # Map
    planning_data   = PlanningData()

    first_it        = 1
    Pub_counter     = 0
    it_counter      = 0
    # Initialize variables for main loop
    GlobalState     = np.zeros(6)
    LocalState      = np.zeros(6)
    RunController   = 1
    Counter         = 0
    uApplied        = np.array([0.0, 0.0])
    oldU            = np.array([0.0, 0.0])

    vel_th          = rospy.get_param("/control/vel_th")
    pi              = np.pi

    vector_length   = 42000
    DATA            = np.zeros((vector_length,8))      # [vx vy psidot thetae s ye vxaccel vyaccel udelta uaccel]
    REFS            = np.zeros((vector_length,1))      # [refVx]
    GLOBAL_DATA     = np.zeros((vector_length,3))       # [x y psi]
    PREDICTED_DATA  = np.zeros((vector_length,120))     # [vx vy psidot thetae s ye] presicted to N steps
    TLAPTIME        = np.zeros((30,1))
    ELAPSD_TIME     = np.zeros((vector_length,1))
    CONTROL_ACTIONS = np.zeros((vector_length,2))
    Hist_pos        = np.zeros((vector_length,4))
    Hist_traj       = np.zeros((vector_length,4))
    IDENT_DATA      = np.zeros((vector_length,5))
    Data_for_RMSE   = np.zeros((vector_length,4))

    vel_ref = np.zeros([N,1])
    curv_ref = np.zeros([N,1])
    y_ref = np.zeros([N,1])
    yaw_ref = np.zeros([N,1])
    x_ref = np.zeros([N,1])

    # Loop running at loop rate
    TimeCounter     = 0
    PlannerCounter  = 0
    count           = True
    index           = 0
    start_LapTimer  = datetime.datetime.now()
    test_gen        = 0# test generated plan
    test_type       = 1# 1=vel cte 0=acc cte
    acc_ref         = 1.5
    dacc_ref        = 0
    velocity        = 0
    flagIni         = True
    rise_flag.data  = True
    wheel_diameter  = 6.3/100

    rospy.sleep(1.2)   # Soluciona los problemas de inicializacion esperando a que el estimador se inicialice bien
    SS          = 0
    ct_avg      = 0

    #maximum values
    v_max     = rospy.get_param("max_vel")
    v_min     = 0.01
    ac_max    = 3.0
    ac_min    = -3.0
    e_max     = 0.5
    e_min     = -0.5
    str_max   = 0.249
    str_min   = -0.249
    dstr_max  = str_max*0.1
    dstr_min  = str_min*0.1
    dac_max   = ac_max*0.1
    dac_min   = ac_min*0.1
    Counter   = 0

    vx_scale  = 1/((v_max-v_min)**2)
    acc_scale = 1/((ac_max-ac_min)**2)
    ey_scale  = 1/((e_max-e_min)**2)
    str_scale = 1/((str_max-str_min)**2)
    dstr_scale = 1/((dstr_max-dstr_min)**2)
    dacc_scale = 1/((dac_max-dac_min)**2)

###----------------------------------------------------------------###
    ### PATH TRACKING TUNING:

    # Q  = 0.8 * np.diag([0.7*vx_scale, 0.0, 0.0, 0.1*ey_scale, 0.0, 0.6*ey_scale])
    # R  = 0.05* np.diag([0.15*str_scale,0.1*acc_scale])  # delta, a
    # dR = 0.15 * np.array([0.001*dstr_scale,0.01*dacc_scale])  # Input rate cost u

    Q  = np.diag([120.0, 1.0, 1.0, 70.0, 0.0, 1500.0])   #[vx ; vy ; psiDot ; e_psi ; s ; e_y]
    R  = 1 * np.diag([3, 0.8])                         #[delta ; a]
    dR = 15 * np.array([1.0, 1.5])                        #Input rate cost u

    # #ref mapa
    # Q  = 0.8 * np.diag([0.7*vx_scale, 0.0, 0.0, 0.3*ey_scale, 0.0, 0.8*ey_scale])
    # R  = 0.05* np.diag([0.05*str_scale,0.1*acc_scale])  # delta, a
    # dR = 0.15 * np.array([0.0005*dstr_scale,0.01*dacc_scale])  # Input rate cost u

    Controller  = PathFollowingLPV_MPC(Q, R, dR, N, Vx_ref, dt,map, "OSQP", Steering_Delay, Velocity_Delay)

###----------------------------------------------------------------###

    L_LPV_States_Prediction = np.zeros((N,6))
    LPV_States_Prediction   = np.zeros((N,6))

    pub_flag.publish(rise_flag)
    print("Controller is running")

    while (not rospy.is_shutdown()) and RunController == 1: 

        # Read Measurements
        ct_start = datetime.datetime.now() 
        GlobalState[:] = estimatorData.CurrentState  # The current estimated state vector [vx vy w x y psi]
        LocalState[:]  = estimatorData.CurrentState  # [vx vy w x y psi]
        # print LocalState
        # print LocalState[3::]
        GLOBAL_DATA[TimeCounter,:] = LocalState[3::]
        if test_gen == 0: # wait until the plan is online

            LocalState[4], LocalState[5], LocalState[3], insideTrack = map.getLocalPosition(GlobalState[3], GlobalState[4], GlobalState[5])

            if (LocalState[4] <= 3*map.TrackLength/4):

                for i in range(0,N):

                    if i == N-1:
                        vel_ref[i]  = vel_ref[i-1] + acc_ref*dt

                        if vel_ref[i] > Vx_ref:
                            vel_ref[i] = Vx_ref
                                              
                    else:
                        vel_ref[i] = vel_ref[i+1]

            else:

                for i in range(0,N):

                    if i == N-1:

                        vel_ref[i,0]  = vel_ref[i-1,0] + dacc_ref*dt

                        if vel_ref[i,0] <= 0:
                            vel_ref[i,0] = 0                 
                            
                    else:
                        vel_ref[i,0] = vel_ref[i+1,0]  
            REFS[TimeCounter,0] =  vel_ref[0,0]

        elif test_gen == 1:

            if test_type:
                for i in range(0,N):
                    vel_ref = Vx_ref*np.ones([N,1])
                    if i == N-1:
                        x_ref[i,0] = x_ref[i-1,0] + vel_ref[i,0]*dt
                    else:
                        x_ref[i,0] = x_ref[i+1,0]
            else:
                for i in range(0,N):

                    if i == N-1:
                        vel_ref[i,0]  = vel_ref[i-1,0] + acc_ref*dt
                        if vel_ref[i,0]>2.5:
                            # vel_ref[i,0]=0.5 vel top
                            acc_ref = - acc_ref

                        if vel_ref[i,0]<0:
                            # vel_ref[i,0]=0.5 vel top
                            vel_ref[i,0] = 0
                            acc_ref = - acc_ref
                            
                        x_ref[i,0]    = x_ref[i-1,0] + vel_ref[i,0]*dt
                    else:
                        vel_ref[i,0] = vel_ref[i+1,0]
                        x_ref[i,0]   = x_ref[i+1,0]

        startTimer = datetime.datetime.now()

        oldU = uApplied
        uApplied = np.array([cmd.servo, cmd.motor])

        Controller.OldSteering.append(cmd.servo) # meto al final del vector
        Controller.OldAccelera.append(cmd.motor)
        Controller.OldSteering.pop(0)
        Controller.OldAccelera.pop(0)

        Hist_pos[TimeCounter,:]   = [LocalState[3], LocalState[4], wrap(LocalState[5]), LocalState[0] ]
        Hist_traj[TimeCounter,:]  = [x_ref[0,0], y_ref[0,0], wrap(yaw_ref[0,0]), vel_ref[0,0] ]        

        GlobalState[5] = wrap(GlobalState[5])     

        # print("vel_ref:  ", vel_ref[0,0],"actual_vel:  ",LocalState[0], "curv", curv_ref[0,0] )
        # print("x_ref:  ", x_ref[0,0],"y_ref:  ",y_ref[0,0], "yaw_ref", wrap(yaw_ref[0,0]) )
        # print("x:  ", LocalState[3],"y:  ",LocalState[4], "yaw", wrap(LocalState[5]) )

        # LocalState[4], Xerror, LocalState[5], LocalState[3] = Body_Frame_Errors(GlobalState[3],
        #  GlobalState[4], GlobalState[5], x_ref[0,0], y_ref[0,0], wrap(yaw_ref[0,0]), SS, LocalState[0],
        #  LocalState[1], curv_ref[0,0], dt )


        SS = LocalState[4]  # Aunque parezca que no, esto es necesario
        ###################################################################################################
        ###################################################################################################

        if first_it < 2:
            # print('Iteration = ', first_it)
            vel_ref     = 0*np.ones([N,1])
            accel_rate = 0.0

            # if vel_ref[0,0] < vel_th or LocalState[0] < vel_th:

            #     add = max(2.5-LocalState[0],max(2.5-vel_ref))
            #     LocalState[0] += add
            #     vel_ref += add

            # xx, uu      = predicted_vectors_generation(N, LocalState, accel_rate, dt)
            xx, uu      = predicted_vectors_generation_V2(N, LocalState, accel_rate, dt)
            Controller.solve(LocalState[0:6], xx, uu, NN_LPV_MPC, vel_ref,curv_ref, 0, 0, 0, first_it)
            # print first_it
            first_it    = first_it + 1


            Controller.OldPredicted = np.hstack((Controller.OldSteering[0:len(Controller.OldSteering)-1], Controller.uPred[Controller.steeringDelay:Controller.N,0]))
            Controller.OldPredicted = np.concatenate((np.matrix(Controller.OldPredicted).T, np.matrix(Controller.uPred[:,1]).T), axis=1)

        else:

            NN_LPV_MPC  = False
            # print vel_ref
            LPV_States_Prediction, A_L, B_L, C_L = Controller.LPVPrediction(LocalState[0:6], Controller.uPred, vel_ref, curv_ref)
            # print("Errors",LocalState[0:6])
            Controller.solve(LPV_States_Prediction[0,:], LPV_States_Prediction, Controller.uPred, NN_LPV_MPC, vel_ref, curv_ref, A_L, B_L, C_L, first_it)

            # print('---> Controller.uPred = ', Controller.uPred[0,:])
        ###################################################################################################
        ###################################################################################################

        if Counter >-1:
            if first_it > 19:
                new_LPV_States_Prediction = LPV_States_Prediction[0, :]
                for i in range(1,N):
                    new_LPV_States_Prediction = np.hstack((new_LPV_States_Prediction, LPV_States_Prediction[i,:]))
                PREDICTED_DATA[Counter,:] = new_LPV_States_Prediction

            # # Model delay with an small horizon? 

            CONTROL_ACTIONS[TimeCounter,:] = [cmd.servo, cmd.motor]
            DATA[TimeCounter,:]   = np.hstack((LocalState, uApplied))

            endTimer = datetime.datetime.now();
            deltaTimer = endTimer-ct_start

            ELAPSD_TIME[Counter,:] = deltaTimer.total_seconds()

            # ClosedLoopData.addMeasurement(GlobalState, LocalState, uApplied, Counter, deltaTimer.total_seconds())
            # Data_for_RMSE[TimeCounter,:] = [ LocalState[0], LocalState[5], LocalState[3], LocalState[7]]

            # Publishing important info about the racing:

            TimeCounter     += 1
            Pub_counter     += 1 
            
            ## Publish input simulation IRI ##
            cmd.servo = Controller.uPred[0,0];
            cmd.motor = Controller.uPred[0,1];

            ## Publish input car IDIADA
            ### Parse from cmd to cmd_car -> IDIADA

            # cmd_car.left_steer_turn_radius_reciprocal_command=u_steer
            #
            # if  u_acc >= 0:
            #     cmd_car.acceleration_command = u_acc
            #     cmd_car.brake_deceleration_command = 0
            # else:
            #     cmd_car.acceleration_command = 0
            #     cmd_car.brake_deceleration_command = -u_acc

            ### Publish input ###
            # Pub_counter     += 1

            # if Pub_counter == 3:
            #     input_commands.publish(cmd_car)
            #     Pub_counter = 0
            

            #if Pub_counter == 3:
            
            Pub_counter = 0
            PlannerCounter  += 1
            '''print cmd.motor
            print cmd.servo'''
            if cmd.motor is None:
            	cmd.motor = 0
            if cmd.servo is None:
            	cmd.servo = 0

            input_commands.publish(cmd)
            velocity += cmd.motor * dt
            inputVel  = (velocity/(wheel_diameter/2)) * (60/(2*np.pi))
            inputVel  = sorted([-1000, inputVel, 1000])[1]
            inputSter = 90 - 1.3*((cmd.servo*180)/np.pi)
            controlVel_commands.publish(inputVel)
            controlSter_commands.publish(inputSter)


        #input_commands.publish(cmd)
        Counter  += 1
        #print("elapsed time",deltaTimer.total_seconds())
        rate.sleep()

    # END WHILE
    '''
    plt.figure(2)
    plt.subplot(611)
    plt.plot(CONTROL_ACTIONS[0:TimeCounter,0], '-')
    plt.legend(['Steering'], loc='best')
    plt.grid()

    plt.subplot(612)
    plt.plot(CONTROL_ACTIONS[0:TimeCounter,1], '-')
    plt.legend(['Acceleration'], loc='best')
    plt.grid()

    plt.subplot(613)
    plt.plot(Hist_traj[0:TimeCounter,1], '.')
    # plt.legend([], loc='best') #Hist_pos[:,0],Hist_pos[:,1],Hist_traj[:,0],Hist_traj[:,1]
    plt.subplot(613)
    plt.plot(Hist_pos[0:TimeCounter,1], '.')
    plt.legend(['Trajectory Y',' Y'], loc='best')
    plt.grid()

    plt.subplot(614)
    plt.plot(Hist_traj[0:TimeCounter,3], '.')
    # plt.legend([], loc='best') #Hist_pos[:,0],Hist_pos[:,1],Hist_traj[:,0],Hist_traj[:,1]
    plt.subplot(614)
    plt.plot(Hist_pos[0:TimeCounter,3], '.')
    plt.legend(['Trajectory Vel',' Vel'], loc='best')
    plt.grid()

    plt.subplot(615)
    plt.plot(Hist_traj[0:TimeCounter,2], 'r.')
    #plt.legend([], loc='best') #Hist_pos[:,0],Hist_pos[:,1],Hist_traj[:,0],Hist_traj[:,1]
    plt.subplot(615)
    plt.plot(Hist_pos[0:TimeCounter,2], 'g.')
    plt.legend(['Trajectory yaw','yaw'], loc='best')
    plt.grid()

    plt.subplot(616)
    plt.plot(DATA[0:TimeCounter,5], '-')
    plt.legend(['Lateral error'], loc='best')
    plt.grid()

    plt.show()

    day         = '28_1_2019'
    num_test    = 'ResultsPaper'
    newpath     = '/home/marc/Escritorio/results_simu_test/'+day+'/'+num_test+'/'
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    np.savetxt(newpath+'/DATA2.dat', DATA, fmt='%.5e')
    np.savetxt(newpath+'/REFS2.dat', REFS, fmt='%.5e') 
    np.savetxt(newpath+'/GLOBAL_DATA2.dat', GLOBAL_DATA, fmt='%.5e')
'''
    quit()










# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================

def Body_Frame_Errors (x, y, psi, xd, yd, psid, s0, vx, vy, curv, dt):

    ex = (x-xd)*np.cos(psid) + (y-yd)*np.sin(psid)

    ey = -(x-xd)*np.sin(psid) + (y-yd)*np.cos(psid)

    epsi = wrap(psi - psid)

    #s = s0 + np.sqrt(vx*vx + vy*vy) * dt
    s = s0 + ( (vx*np.cos(epsi) - vy*np.sin(epsi)) / (1-ey*curv) ) * dt



    return s, ex, ey, epsi



def predicted_vectors_generation(Hp, LocalState, accel_rate, Ts):

    L_f     = rospy.get_param("lf")
    L_r     = rospy.get_param("lr")
    m       = rospy.get_param("m")
    I_z     = rospy.get_param("Iz")
    Cf      = rospy.get_param("Cf")
    Cr      = rospy.get_param("Cr")
    mu      = rospy.get_param("mu")

    vx      = LocalState[0]
    vy      = LocalState[1]
    psiDot  = LocalState[2]

    Accel = 0.1

    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):

        a_F = 0.0 - arctan((vy + L_f*psiDot)/abs(vx))
        a_R = arctan((- vy + L_r*psiDot)/abs(vx))

        FyF = Cf * a_F
        FyR = Cr * a_R

        ax      = Accel[i] - mu*vx
        ay      = 1.0/m*(FyF*cos(0.0)+FyR)
        vx      += Ts*(ax + psiDot*vy)
        vy      += Ts*(ay - psiDot*vx)
        psiDot  += Ts*(1.0/I_z*(L_f*FyF*cos(0.0) - L_r*FyR))

        s       +=Ts*vx

        ## [vx vy omega theta_e s y_e]
        xx[i,:] = [vx, vy, psiDot, 0.00001, s, 0.001]

        uu[i,:] = [0.0, Accel[i]]

    return xx, uu



def predicted_vectors_generation_V2(Hp, x0, accel_rate, dt):

    Vx      = np.zeros((Hp+1, 1))
    Vx[0]   = x0[0]
    S       = np.zeros((Hp+1, 1))
    S[0]    = 0
    Vy      = np.zeros((Hp+1, 1))
    Vy[0]   = x0[1]
    W       = np.zeros((Hp+1, 1))
    W[0]    = x0[2]
    Ey      = np.zeros((Hp+1, 1))
    Ey[0]   = x0[3]
    Epsi    = np.zeros((Hp+1, 1))
    Epsi[0] = x0[4]

    Accel   = 1.0
    curv    = 0

    for i in range(0, Hp):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[3]
        Epsi[i+1]    = x0[4]

    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt

    xx  = np.hstack([ Vx, Vy, W, Epsi ,S ,Ey]) # [vx vy omega theta_e s y_e]
    uu = np.zeros(( Hp, 1 ))
    return xx, uu



def plotTrajectory(map, ClosedLoop, Complete_Vel_Vect):
    x = ClosedLoop.x
    x_glob = ClosedLoop.x_glob
    u = ClosedLoop.u
    time = ClosedLoop.SimTime
    it = ClosedLoop.iterations
    elapsedTime = ClosedLoop.elapsedTime
    #print elapsedTime

    # plt.figure(3)
    # plt.plot(time[0:it], elapsedTime[0:it, 0])
    # plt.ylabel('Elapsed Time')
    # ax = plt.gca()
    # ax.grid(True)

    plt.figure(2)
    plt.subplot(711)
    plt.plot(time[0:it], x[0:it, 0], color='b', label='Response')
    plt.plot(time[0:it], Complete_Vel_Vect[0:it], color='r', label='Reference')
    plt.ylabel('vx')
    ax = plt.gca()
    ax.legend()
    ax.grid(True)
    plt.subplot(712)
    plt.plot(time[0:it], x[0:it, 1])
    plt.ylabel('vy')
    ax = plt.gca()
    ax.grid(True)
    plt.subplot(713)
    plt.plot(time[0:it], x[0:it, 2])
    plt.ylabel('wz')
    ax = plt.gca()
    ax.grid(True)
    plt.subplot(714)
    plt.plot(time[0:it], x[0:it, 3],'k')
    plt.ylabel('epsi')
    ax = plt.gca()
    ax.grid(True)
    plt.subplot(715)
    plt.plot(time[0:it], x[0:it, 5],'k')
    plt.ylabel('ey')
    ax = plt.gca()
    ax.grid(True)
    plt.subplot(716)
    plt.plot(time[0:it], u[0:it, 0], 'r')
    plt.ylabel('steering')
    ax = plt.gca()
    ax.grid(True)
    plt.subplot(717)
    plt.plot(time[0:it], u[0:it, 1], 'r')
    plt.ylabel('acc')
    ax = plt.gca()
    ax.grid(True)
    plt.show()



if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
