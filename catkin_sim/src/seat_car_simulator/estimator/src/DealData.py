#!/usr/bin/env python

from gazebo_msgs.msg import LinkState, ModelState
from estimator.srv import lmi_data, lmi_dataResponse
from lpv_mpc.msg import simulatorStates,pos_info
from estimator.msg import ErrorsInfo,DataInfo,SendCov,LandmarkInfo,LandmarksInfo
from estimator.msg import Gains

import scipy.io as sio
import numpy as np
import os
import rospy

'''

Node used to subscrive to the set of topics that send information after a simulation and store it in your PC, also used
to load matrices into the system.

'''

#Set path to save data

str         = 'SLAM_ZKF_NEW_EKF'
num_test    = 'ResultsNewModelEKF'
newpath     = '/home/marc/Desktop/paper2022/paper2022_lessLand/'+str+'/'+num_test+'/'

# Load gain matrices from diferent paths and set up the saving folder

def handle_load_data(req):

    if req.est_id == "LPV":

        #str = req.est_id

        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/LPV'
        Est_Gains = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['SchedVars_Limits']

    elif req.est_id == "LPV_UIO":

        str = req.est_id

        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/LPV_UIO'
        Est_Gains = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['SchedVars_Limits']

    elif req.est_id == "LPV_SLAM":

        str = req.est_id

        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/LPV_SLAM'
        Est_Gains = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['SchedVars_Limits']


    if req.est_id == "ZKF":

        #str = req.est_id

        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/ZKF'
        Est_Gains = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['SchedVars_Limits']

    elif req.est_id == "ZKF_UIO":

        #str = req.est_id

        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/ZKF_UIO'
        Est_Gains = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['SchedVars_Limits']

    elif req.est_id == "ZKF_SLAM":

        str = req.est_id

        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/ZKF_SLAM'
        Est_Gains = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/Estimator_Gains_HS.mat')['SchedVars_Limits']

    elif req.est_id == "LMI_din":

        str = req.est_id
        print("LMI_din")
        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/LPV_NV'
        Est_Gains = sio.loadmat(estimator_path + '/LMI_din.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/LMI_din.mat')['SchedVars_Limits']

    elif req.est_id == "LMI_cin":

        str = req.est_id
        print("LMI_cin")
        estimator_path = '/home/marc/sim_IRI/catkin_sim/src/seat_car_simulator/lpv_control/lpv_mpc/src/data/OBSERVER_GAIN/LPV_NV'
        Est_Gains = sio.loadmat(estimator_path + '/LMI_cin.mat')['Llmi']  # vy max = 0.3 / w max = 3
        SchedVars_Limits = sio.loadmat(estimator_path + '/LMI_cin.mat')['SchedVars_Limits']

    # Reshape gain matrices so that they can be set using ROS msgs and loaded in Cpp

    resp = lmi_dataResponse()
    shape_gains = np.shape(Est_Gains)

    print shape_gains
    shape_schd  = np.shape(SchedVars_Limits)

    for j in range(0, shape_gains[2]):

        aux = Gains()

        for k in range(0, shape_gains[0]):

            for l in range(0, shape_gains[1]):

                aux.gains.append(float(Est_Gains[k,l,j]))

        resp.L.append(aux)

    for i in range(0,shape_schd[0]):

        resp.limits.min.append(SchedVars_Limits[i,0])
        resp.limits.max.append(SchedVars_Limits[i,1])

    return resp

############################################################################################
######################Set of classes used to sub and save data Inter #######################
############################################################################################

class SensorInfo(object):

    def __init__(self):
        rospy.Subscriber('sensorStates', simulatorStates, self.sensor_callback, queue_size=1)
        self.sensor = []
        self.folder = []

    def sensor_callback(self,msg):
        if msg.vx >1.0:
            self.sensor.append([msg.vx, msg.psiDot, msg.x, msg.y, msg.psi])

    def save_data(self):

        if not os.path.exists(newpath):
            os.makedirs(newpath)

        np.save(newpath+'/'+'SENSOR', self.sensor)

class ErrorsCar(object):

    def __init__(self):
        rospy.Subscriber('errors', ErrorsInfo, self.errors_callback, queue_size=1)
        self.ERRORS = []
        self.RMSE   = []

    def errors_callback(self,msg):

        for lec in msg.err:
            self.ERRORS.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        self.RMSE = msg.rmse

        if msg.folder == "":
            msg.folder = str

        if not os.path.exists(newpath+msg.folder+'/'):
            os.makedirs(newpath+msg.folder+'/')

        np.save(newpath+msg.folder+'/'+'ERRORS', self.ERRORS)
        np.save(newpath+msg.folder+'/'+'RMSE', self.RMSE)

class CovCar(object):

    def __init__(self):
        rospy.Subscriber('Cov', SendCov, self.cov_callback, queue_size=1)
        self.COV = []

    def cov_callback(self,msg):

        self.COV.append(msg.matrix)

        if msg.folder == "":
            msg.folder = str

        if not os.path.exists(newpath+msg.folder+'/'):
            os.makedirs(newpath+msg.folder+'/')

        np.save(newpath+msg.folder+'/'+'COV', self.COV)

class LandInfo(object):

    def __init__(self):
        rospy.Subscriber('LandmarkData', LandmarksInfo, self.LandmarkInfo, queue_size=1)
        self.Landmark = []

    def LandmarkInfo(self,msg):

        for j in range(0,len(msg.lnd_list)):

            for i in range(0, len(msg.lnd_list[j].x)):

                self.Landmark.append([j,msg.lnd_list[j].x[i], msg.lnd_list[j].y[i], msg.lnd_list[j].id[i]])

        if msg.lnd_list[0].folder == "":
            msg.lnd_list[0].folder = str

        if not os.path.exists(newpath+ msg.lnd_list[0].folder+'/'):
            os.makedirs(newpath+ msg.lnd_list[0].folder+'/')

        np.save(newpath+ msg.lnd_list[0].folder+'/'+'Landmark', self.Landmark)

class LandInfoRaw(object):

    def __init__(self):
        rospy.Subscriber('LandmarkData_raw', LandmarksInfo, self.LandInfoRaw, queue_size=1)
        self.Landmark = []

    def LandInfoRaw(self,msg):

        for j in range(0,len(msg.lnd_list)):

            for i in range(0, len(msg.lnd_list[j].x)):

                self.Landmark.append([j,msg.lnd_list[j].x[i], msg.lnd_list[j].y[i], msg.lnd_list[j].id[i]])

        if msg.lnd_list[0].folder == "":
            msg.lnd_list[0].folder = str

        if not os.path.exists(newpath+ msg.lnd_list[0].folder+'/'):
            os.makedirs(newpath+ msg.lnd_list[0].folder+'/')

        np.save(newpath+ msg.lnd_list[0].folder+'/'+'LandmarkRaw', self.Landmark)

class DataCar(object):

    def __init__(self):
        rospy.Subscriber('data', DataInfo, self.data_callback, queue_size=1)
        self.EST      = []
        self.GROUND   = []
        self.MIN      = []
        self.MAX      = []
        self.MIN_DIST = []
        self.MAX_DIST = []

    def data_callback(self,msg):

        i_interv = 0
        i_dist = 0

        for lec in msg.data:
            self.EST.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        for lec in msg.ground:
            self.GROUND.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        for lec in msg.max_limits:
            self.MIN.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])
            i_interv += 1

        for lec in msg.min_limits:
            self.MAX.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        for lec in msg.max_dist:
            self.MIN_DIST.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])
            i_dist += 1

        for lec in msg.min_dist:
            self.MAX_DIST.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        if not os.path.exists(newpath):
            os.makedirs(newpath)

        if msg.folder == "":
            msg.folder = str

        np.save(newpath+msg.folder+'/'+'DATA', self.EST)
        np.save(newpath+'/GROUND', self.GROUND)


        if (i_interv!=0):
            np.save(newpath+msg.folder+'/'+'MAX', self.MIN)
            np.save(newpath+msg.folder+'/'+'MIN', self.MAX)

        if (i_dist!=0):
            np.save(newpath+msg.folder+'/'+'MAX_DIST', self.MAX_DIST)
            np.save(newpath+msg.folder+'/'+'MIN_DIST', self.MIN_DIST)

############################################################################################
######################Set of classes used to sub and save data EKF #######################
############################################################################################

class ErrorsCarEKF(object):

    def __init__(self):
        rospy.Subscriber('errorsEKF', ErrorsInfo, self.errors_callback, queue_size=1)
        self.ERRORS = []
        self.RMSE   = []

    def errors_callback(self,msg):

        for lec in msg.err:
            self.ERRORS.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        self.RMSE = msg.rmse

        if msg.folder == "":
            msg.folder = str

        if not os.path.exists(newpath+msg.folder+'/'):
            os.makedirs(newpath+msg.folder+'/')

        np.save(newpath+msg.folder+'/'+'ERRORS_EKF', self.ERRORS)
        np.save(newpath+msg.folder+'/'+'RMSE_EKF', self.RMSE)

class CovCarEKF(object):

    def __init__(self):
        rospy.Subscriber('Cov', SendCov, self.cov_callback, queue_size=1)
        self.COV = []

    def cov_callback(self,msg):

        self.COV.append(msg.matrix)

        if msg.folder == "":
            msg.folder = str

        if not os.path.exists(newpath+msg.folder+'/'):
            os.makedirs(newpath+msg.folder+'/')

        np.save(newpath+msg.folder+'/'+'COV', self.COV)

class LandInfoEKF(object):

    def __init__(self):
        rospy.Subscriber('LandmarkDataEKF', LandmarksInfo, self.LandmarkInfo, queue_size=1)
        self.Landmark = []

    def LandmarkInfo(self,msg):

        for j in range(0,len(msg.lnd_list)):

            for i in range(0, len(msg.lnd_list[j].x)):

                self.Landmark.append([j,msg.lnd_list[j].x[i], msg.lnd_list[j].y[i], msg.lnd_list[j].id[i]])

        if msg.lnd_list[0].folder == "":
            msg.lnd_list[0].folder = str

        if not os.path.exists(newpath+ msg.lnd_list[0].folder+'/'):
            os.makedirs(newpath+ msg.lnd_list[0].folder+'/')

        np.save(newpath+ msg.lnd_list[0].folder+'/'+'LandmarkEKF', self.Landmark)

class LandInfoRawEKF(object):

    def __init__(self):
        rospy.Subscriber('LandmarkData_rawEKF', LandmarksInfo, self.LandInfoRaw, queue_size=1)
        self.Landmark = []

    def LandInfoRaw(self,msg):

        for j in range(0,len(msg.lnd_list)):

            for i in range(0, len(msg.lnd_list[j].x)):

                self.Landmark.append([j,msg.lnd_list[j].x[i], msg.lnd_list[j].y[i], msg.lnd_list[j].id[i]])

        if msg.lnd_list[0].folder == "":
            msg.lnd_list[0].folder = str

        if not os.path.exists(newpath+ msg.lnd_list[0].folder+'/'):
            os.makedirs(newpath+ msg.lnd_list[0].folder+'/')

        np.save(newpath+ msg.lnd_list[0].folder+'/'+'LandmarkRawEKF', self.Landmark)

class DataCarEKF(object):

    def __init__(self):
        rospy.Subscriber('dataEKF', DataInfo, self.data_callback, queue_size=1)
        self.EST      = []
        self.GROUND   = []
        self.MIN      = []
        self.MAX      = []
        self.MIN_DIST = []
        self.MAX_DIST = []

    def data_callback(self,msg):

        i_interv = 0
        i_dist = 0

        for lec in msg.data:
            self.EST.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        for lec in msg.ground:
            self.GROUND.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        for lec in msg.max_limits:
            self.MIN.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])
            i_interv += 1

        for lec in msg.min_limits:
            self.MAX.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        for lec in msg.max_dist:
            self.MIN_DIST.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])
            i_dist += 1

        for lec in msg.min_dist:
            self.MAX_DIST.append([lec.vx, lec.vy, lec.psiDot, lec.x, lec.y, lec.psi ])

        if not os.path.exists(newpath):
            os.makedirs(newpath)

        if msg.folder == "":
            msg.folder = str

        np.save(newpath+msg.folder+'/'+'DATA_EKF', self.EST)
        np.save(newpath+'/GROUND_EKF', self.GROUND)


        if (i_interv!=0):
            np.save(newpath+msg.folder+'/'+'MAX_EKF', self.MIN)
            np.save(newpath+msg.folder+'/'+'MIN_EKF', self.MAX)

        if (i_dist!=0):
            np.save(newpath+msg.folder+'/'+'MAX_DIST_EKF', self.MAX_DIST)
            np.save(newpath+msg.folder+'/'+'MIN_DIST_EKF', self.MIN_DIST)

############################################################################################
#########################Set of classes used to sub and save data ##########################
############################################################################################

def main():
    rospy.init_node('MatrixLoader')
    s = rospy.Service('LoadMatrices', lmi_data, handle_load_data)
    rospy.Rate(200)
    saver = ErrorsCar()
    data  = DataCar()
    cov   = CovCar()
    lnd   = LandInfo()
    lnd_raw   = LandInfoRaw()
    sns   = SensorInfo()
    ErrorsCarEKF()
    cov_ekf = CovCarEKF()
    land_ekf = LandInfoEKF()
    land_raw_ekf = LandInfoRawEKF()
    cae_ekf =DataCarEKF()
    while not rospy.is_shutdown():
        pass
    print("saving DATA")
    sns.save_data()
    quit()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass
