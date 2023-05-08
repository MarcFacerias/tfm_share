
#define _GLIBCXX_USE_C99 1
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <iterator>
#include <vector>
#include <sstream>
#include <deque>
#include <string>
#include <sstream>
#include <l4vehicle_msgs/VehicleState.h>
#include <lpv_mpc/ECU.h>
#include <lpv_mpc/pos_info.h>
#include <lpv_mpc/simulatorStates.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <ctime>
#include <math.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <estimator/lmi_data.h>
#include <estimator/ErrorStates.h>
#include <estimator/ErrorsInfo.h>
#include <estimator/LandmarkInfo.h>
#include <estimator/DataInfo.h>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include<Eigen/Core>
#include<Eigen/SVD>
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
                                // GLOBAL //
///////////////////////////////////////////////////////////////////////////////

/* Node used to test the SLAM without any estimation,
 it was used in early stages of the developement and might require an update*/

float dt;
bool flag = 0;
float g_x;
float g_y;

///////////////////////////////////////////////////////////////////////////////
                             // CALLBACK and AUX //
///////////////////////////////////////////////////////////////////////////////
struct PolarPose{

	float range;
	float angle;
	uint id;

};

struct GlobalPose{

	float x;
	float y;
	uint id;

};

struct lnd_sample
{

  GlobalPose lnd_read;
  float x_robot;
  float y_robot;
  int time_stamp;
  
};

float wrap(float angle){

  /*while (angle < -M_PI){

    angle += M_PI;

  }

  while (angle > M_PI){

    angle -= M_PI;

  }*/

  return angle;

}


void Polar(float x, float y, PolarPose *pose){


	pose -> range     = sqrt(x*x + y*y);
	pose -> angle     = atan2(y, x);

 /* pose -> range     = x;
  pose -> angle     = y;*/

}

////////////////////////////classes////////////////////////////
class GetMarkers{

  private:

    ros::NodeHandle n;
    ros::Subscriber marker_sub;

  public:

  	std::deque<PolarPose> landmark_list;
    std::vector<GlobalPose> landmark_list_gb;

    void GetMarkersStart(){

      marker_sub = n.subscribe("/Corrected_Pose", 1, &GetMarkers::GetMarkersCallback, this);

    }

    void GetMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
    {

    	for (int i = 0; i < msg.markers.size(); i++){

    		PolarPose pose;

    		Polar(msg.markers[i].pose.pose.position.x, msg.markers[i].pose.pose.position.y, &pose);
    		pose.id = msg.markers[i].id;
    		landmark_list.push_back(pose);

        GlobalPose pose_g;
        pose_g.x  = msg.markers[i].pose.pose.position.x;
        pose_g.y  = msg.markers[i].pose.pose.position.y;
        pose_g.id = msg.markers[i].id;
        landmark_list_gb.push_back(pose_g);        

    	}
    }

};

class GetSensorData{

  public:
    float x = 0.3;
    float y = 0;
    float yaw = 0;
    float psiDot = 0;
    float vx = 0;
    ros::NodeHandle n;
    ros::Subscriber pose_sub;

    GetSensorData(){

      pose_sub = n.subscribe("/sensorStates", 1, &GetSensorData::GetPoseCallback, this);

    }

    void GetPoseCallback(const lpv_mpc::simulatorStates& msg)
    {

        x       = msg.x;
        y       = msg.y;
        yaw     = wrap(msg.psi);
        vx      = msg.vx;
        psiDot  = msg.psiDot;

    }

};

class GetGroundTruthData{

  public:
    float x = 0;
    float y = 0;
    float yaw = 0;
    float psiDot = 0;
    float vx = 0;
    float vy = 0;

    std::list <float> x_hist;
    std::list <float> y_hist;
    std::list <float> vx_hist;
    std::list <float> vy_hist;
    std::list <float> yaw_hist;
    std::list <float> psiDot_hist;
    ros::NodeHandle n;
    ros::Subscriber groundTruth_sub;

    GetGroundTruthData(){

      groundTruth_sub = n.subscribe("/vehicle_state", 1, &GetGroundTruthData::GetGroundTruthDataCallback, this);

    }

    void GetGroundTruthDataCallback(const l4vehicle_msgs::VehicleState& msg)
    {

        x       = msg.x;
        y       = msg.y;
        yaw     = wrap(msg.heading);
        vx      = msg.longitudinal_velocity;
        vy      = msg.lateral_velocity;
        psiDot  = msg.angular_velocity;

        x_hist.push_back(x);
        y_hist.push_back(y);
        yaw_hist.push_back(yaw);
        vx_hist.push_back(vx);
        vy_hist.push_back(vy);
        psiDot_hist.push_back(psiDot);

    }

};

class GetActuData{

  public:

    float a  = 0.0;
    float steer = 0.0;
    ros::Subscriber actu_sub;
    ros::NodeHandle n;

    GetActuData(){

      actu_sub = n.subscribe("/ecu", 1, &GetActuData::GetActuCallback, this);

    }

    private:

    void GetActuCallback(const lpv_mpc::ECU& msg)
    {

        a     = msg.motor;
        steer = msg.servo;

    }

};

void FlagCallback(const std_msgs::Bool& msg){

  flag = msg.data;

}


/////////////////////////functions//////////////////////////

VectorXf sort_indexes(const VectorXf v) {

  // initialize original index locations
  VectorXf idx = VectorXf::LinSpaced(v.size()+1, 0, v.size());

  std::stable_sort(idx.data(), idx.data() + idx.size()-1,
       [v](int i1, int i2) {return v(i1) < v(i2);});

  return idx;
}

MatrixXf EnvBox(MatrixXf Rxxio){

  int rows = Rxxio.rows();
  MatrixXf limitMatrix = MatrixXf::Zero(rows,rows);

  for (int i=0; i < rows; i++){

      for (int j=0; j < Rxxio.cols(); j++ ){

            limitMatrix(i,i) += fabs(Rxxio(i,j));

        }
  }

  return limitMatrix;
}

///////////////////////////////////////////////////////////////////////////////
                                // ESTIMATOR //
///////////////////////////////////////////////////////////////////////////////

class ObserverLPV_SLAM{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    ros::Publisher landmark_pub = n.advertise<estimator::LandmarkInfo>("LandmarkData", 5);
    estimator::lmi_data srv;

    // model dimension variables
    float n_states = 10;
    float n_outputs = 7;
    float n_control = 2;

    //general usage variables
    MatrixXf eye6 = MatrixXf::Identity(n_states, n_states);

    //vehicle variables

    float lf;
    float lr;
    float m;
    float I;
    float Cf;
    float Cr;
    float mu;
    float et = 0;

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf L;

    //Vectors
    VectorXf Ew;
    VectorXf Ev;
    VectorXf u;
    VectorXf mu_sch;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<std::vector<float>> sched_vars;

    //DATA from landmarks
    std::deque<GlobalPose> lm_data;

    //estimated states
    float x      = 0.02;
    float y      = 0.0;
    float vx     = 0.75;
    float vy     = 0.0;
    float yaw    = 0.0;
    float psiDot = 0.0;
    float lambda = 0.0;
    VectorXf states_est;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;
    std::list <std::vector<float>> est_error_hist;

  void estimateState(GetSensorData sensor, GetActuData ecu, GetMarkers *landmarks){

    VectorXf y_meas;
    u(0)      = ecu.steer;
    u(1)      = ecu.a;
    bool first_est = 1;
    bool lnd_detected = 0;

  	if (!landmarks->landmark_list.empty())
  	{

      if (lm_data.empty()){

        GlobalPose newPose;
        newPose.id = landmarks->landmark_list.back().id;
        newPose.x  = x + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
        newPose.y  = y + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);

        /*newPose.x  = landmarks->landmark_list.back().range;
        newPose.y  = landmarks->landmark_list.back().angle;*/

        lm_data.push_back(newPose);
        landmarks->landmark_list.pop_back();
      }

  		while (!landmarks->landmark_list.empty() && (lnd_detected == 0)){

  			int it;
  			for (it = 0; it < lm_data.size(); it++){

  				if (lm_data[it].id == landmarks->landmark_list.back().id){

  					lnd_detected = 1;
  					break;

  				}
  			}

  		if (!lnd_detected){

  			GlobalPose newPose;
  			newPose.id = landmarks->landmark_list.back().id;

  			newPose.x  = x + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
  			newPose.y  = y + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);

        /*newPose.x  = landmarks->landmark_list.back().range;
        newPose.y  = landmarks->landmark_list.back().angle;*/


  			lm_data.push_back(newPose);

        landmarks->landmark_list.pop_back();
  			break;

  		}

			else{

				lm_data[it].x  = x + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
				lm_data[it].y  = y + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);

        /*lm_data[it].x  = landmarks->landmark_list.back().range;
        lm_data[it].y  = landmarks->landmark_list.back().angle;*/

			}

      std::cout << "range: " << landmarks->landmark_list.back().angle << std::endl;
      std::cout << "computed range: " << atan2(lm_data[it].y - y, lm_data[it].x - x) + yaw << std::endl;
      std::cout << "yaw: " << yaw << std::endl;
      landmarks->landmark_list.pop_back();
			lnd_detected = 0;
  		}
  	}
      }

    ObserverLPV_SLAM(){

      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
      Ew.resize(n_states);
      Ev.resize(n_outputs);
      u.resize(n_control);
      states_est.resize(n_states);

      n.getParam("lf",lf);
      n.getParam("lr", lr);
      n.getParam("m", m);
      n.getParam("Iz",I);
      n.getParam("Cf",Cf);
      n.getParam("Cr",Cr);
      n.getParam("mu",mu);

      C << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

      A = MatrixXf::Zero(n_states,n_states);

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      u << 0,0;

      states_est << vx, vy, psiDot, x, y, yaw, 0, 0, 0, 0;
      LoadMatrices();
    }

    void publishData(){

      estimator::LandmarkInfo lnd_msg;

      std::cout << lm_data.size() << std::endl;

      for (int i = 0; i < lm_data.size(); i++){

        lnd_msg.x.push_back(lm_data[i].x);
        lnd_msg.y.push_back(lm_data[i].y);
        lnd_msg.id.push_back(lm_data[i].id);

        std::cout << lm_data[i].x << lm_data[i].y << lm_data[i].id << std::endl;

      }

      landmark_pub.publish(lnd_msg);

    }

    private:

    void LoadMatrices(){

      srv.request.est_id = "LPV_SLAM";
      client.call(srv);

      ROS_ERROR_STREAM("Matrix loaded with: " << srv.response.L.size());
      Llmi.resize(srv.response.L.size());

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi[i].resize(n_states,n_outputs);

        for (int rows = 0; rows < n_states; rows++){

          for (int cols = 0; cols < n_outputs; cols++){

            Llmi[i](rows,cols) = srv.response.L[i].gains[rows*(n_states-1) + cols];

          }
        }
      }

      //fill sched_vars

      sched_vars.resize(srv.response.limits.max.size());

      mu_sch.resize(srv.response.L.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars[i].push_back(srv.response.limits.min[i]);
        sched_vars[i].push_back(srv.response.limits.max[i]);

      }

    }

    // void AB_computation(MatrixXf &A, MatrixXf &B, float theta, float steer, float vx, float vy){
    void AB_computation(float steer, float theta_lm, bool first_est){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;


      if (first_est) {

  	      //Update A
	      A(0,0) =  -mu;
	      A(0,1) = (sin(steer) * Cf) / (m*vx);
	      A(0,2) = (sin(steer) * Cf * lf) / (m*vx) + vy;

	      A(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
	      A(1,2) = -(lf * Cf * cos(steer) - lr * Cr) / (m*vx) - vx;

	      A(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
	      A(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);

	      A(3,0) = cos(yaw);
	      A(3,1) = -sin(yaw);

	      A(4,0) = sin(yaw);
	      A(4,1) = cos(yaw);

	      A(5,2) = 1;

      }

      else{

  	      //Update A
	      A(0,0) =  0;
	      A(0,1) =  0;
	      A(0,2) =  0;

	      A(1,1) =  0;
	      A(1,2) =  0;
	      A(2,1) =  0;
	      A(2,2) =  0;

	      A(3,0) =  0;
	      A(3,1) =  0;

	      A(4,0) =  0;
	      A(4,1) =  0;

	      A(5,2) =  0;

      }

      A(8,3) = -lambda*cos(theta_lm)/dt;
      A(8,4) = -lambda*sin(theta_lm)/dt;
      A(8,6) = lambda*cos(theta_lm)/dt;
      A(8,7) = lambda*sin(theta_lm)/dt;
      A(8,8) = (-1)/dt;

      A(9,3) = -lambda*sin(theta_lm)/dt;
      A(9,4) = lambda*cos(theta_lm)/dt;
      A(9,6) = lambda*sin(theta_lm)/dt;
      A(9,7) = -lambda*cos(theta_lm)/dt;
      A(9,9) = (-1)/dt;

      Aln = eye6 + (A*dt);
      Bln = B*dt;
    }

    // void L_computation(MatrixXf &L, float vx, float vy, float theta, float steer){

    void L_computation(float steer, float theta_lm){

      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);
      float M_theta_lm        = (sched_vars[6][1] - theta_lm) / (sched_vars[6][1] - sched_vars[6][0]);

      mu_sch[0]               = M_vx_despl_min *       M_theta_lm            * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[1]               = M_vx_despl_min *       M_theta_lm            * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[2]               = M_vx_despl_min *       M_theta_lm            * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[3]               = M_vx_despl_min *       M_theta_lm            * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[4]               = M_vx_despl_min *       M_theta_lm            * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[5]               = M_vx_despl_min *       M_theta_lm            * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[6]               = M_vx_despl_min *       M_theta_lm            * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[7]               = M_vx_despl_min *       M_theta_lm            * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[8]               = M_vx_despl_min *       (1-M_theta_lm)        * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[9]               = M_vx_despl_min *       (1-M_theta_lm)        * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[10]              = M_vx_despl_min *       (1-M_theta_lm)        * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[11]              = M_vx_despl_min *       (1-M_theta_lm)        * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[12]              = M_vx_despl_min *       (1-M_theta_lm)        * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[13]              = M_vx_despl_min *       (1-M_theta_lm)        * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[14]              = M_vx_despl_min *       (1-M_theta_lm)        * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[15]              = M_vx_despl_min *       (1-M_theta_lm)        * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

      mu_sch[16]               = (1-M_vx_despl_min) *       M_theta_lm                 * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[17]               = (1-M_vx_despl_min) *       M_theta_lm                 * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[18]               = (1-M_vx_despl_min) *       M_theta_lm                 * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[19]               = (1-M_vx_despl_min) *       M_theta_lm                 * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[20]               = (1-M_vx_despl_min) *       M_theta_lm                 * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[21]               = (1-M_vx_despl_min) *       M_theta_lm                 * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[22]               = (1-M_vx_despl_min) *       M_theta_lm                 * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[23]               = (1-M_vx_despl_min) *       M_theta_lm                 * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[24]               = (1-M_vx_despl_min) *       (1-M_theta_lm)             * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[25]               = (1-M_vx_despl_min) *       (1-M_theta_lm)             * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[26]              = (1-M_vx_despl_min) *       (1-M_theta_lm)             * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[27]              = (1-M_vx_despl_min) *       (1-M_theta_lm)             * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[28]              = (1-M_vx_despl_min) *       (1-M_theta_lm)             * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[29]              = (1-M_vx_despl_min) *       (1-M_theta_lm)             * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[30]              = (1-M_vx_despl_min) *       (1-M_theta_lm)             * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[31]              = (1-M_vx_despl_min) *       (1-M_theta_lm)             * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

      MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

      for (int i = 0; i < 15; i++){

          result += mu_sch[i] * Llmi[i];

      }

      L = -result*dt;
    }
};



class ComputeError{

public:

  ros::Publisher error_pub;
  ros::Publisher data_pub;
  ros::NodeHandle n;
  std::vector<estimator::ErrorStates> errors;
  std::vector<float> rmse;

  ComputeError(){

    error_pub = n.advertise<estimator::ErrorsInfo>("errors", 5);
    data_pub = n.advertise<estimator::DataInfo>("data", 5);

  }

    ///////////////////////////////////////////////////////////////////
   //////////////////////////OVERLOADED FUNCTION//////////////////////
  ///////////////////////////////////////////////////////////////////
  void compute_error(GetGroundTruthData truth,ObserverLPV_SLAM estimator){

    estimator::ErrorStates temp;

    temp.vx = truth.vx - estimator.vx;
    temp.vy = truth.vy - estimator.vy;
    temp.psiDot = truth.psiDot - estimator.psiDot;
    temp.x = truth.x - estimator.x;
    temp.y = truth.y - estimator.y;
    temp.psi = truth.yaw - estimator.yaw;

    // std::cout << temp << '\n';
    errors.push_back(temp);
  }

  void compute_rmse(std::vector<estimator::ErrorStates> errors){


    float error_x = 0;
    float error_y = 0;
    float error_psi = 0;
    float error_vx = 0;
    float error_vy = 0;
    float error_psiDot = 0;
    float numel = 0;

    for (int i = 0; i < errors.size(); i++)
      {
        // Access the object through iterator

        error_x      += errors[i].x*errors[i].x;
        error_y      += errors[i].y*errors[i].y;
        error_psi    += errors[i].psi*errors[i].psi;
        error_vx     += errors[i].vx*errors[i].vx;
        error_vy     += errors[i].vy*errors[i].vy;
        error_psiDot += errors[i].psiDot*errors[i].psiDot;

        numel++;
      }

    rmse = { sqrt(error_x/numel), sqrt(error_y/numel), sqrt(error_psi/numel),
             sqrt(error_vx/numel), sqrt(error_vy/numel), sqrt(error_psiDot/numel)};


  }

  void save_error(GetGroundTruthData truth,ObserverLPV_SLAM estimator){

    //Init com
    estimator::ErrorsInfo msg;
    estimator::DataInfo msg_data;
    compute_rmse(errors);
    msg.rmse = rmse;

    //Parse data auxiliar structs
    estimator::ErrorStates aux;

    for (int i = 0; i < errors.size(); i++ ){

      msg.err.push_back(errors[i]);

    }

    while (!truth.x_hist.empty() && !estimator.x_est_hist.empty()){

      aux.x       = truth.x_hist.front();
      aux.y       = truth.y_hist.front();
      aux.psi     = truth.yaw_hist.front();
      aux.vx      = truth.vx_hist.front();
      aux.vy      = truth.vy_hist.front();
      aux.psiDot  = truth.psiDot_hist.front();

      truth.x_hist.pop_front();
      truth.y_hist.pop_front();
      truth.yaw_hist.pop_front();
      truth.vx_hist.pop_front();
      truth.vy_hist.pop_front();
      truth.psiDot_hist.pop_front();

      msg_data.ground.push_back(aux);

      aux.x       = estimator.x_est_hist.front();
      aux.y       = estimator.y_est_hist.front();
      aux.psi     = estimator.yaw_est_hist.front();
      aux.vx      = estimator.vx_est_hist.front();
      aux.vy      = estimator.vy_est_hist.front();
      aux.psiDot  = estimator.psiDot_est_hist.front();

      estimator.x_est_hist.pop_front();
      estimator.y_est_hist.pop_front();
      estimator.yaw_est_hist.pop_front();
      estimator.vx_est_hist.pop_front();
      estimator.vy_est_hist.pop_front();
      estimator.psiDot_est_hist.pop_front();

      msg_data.data.push_back(aux);

    }

    error_pub.publish(msg);
    data_pub.publish(msg_data);

  }
};



/*class SaveLndData{

public: 

  std::deque<lnd_sample> lnd_sample_v; 

  void loadData(GetMarkers &sensor, int it, float x, float y){

  for (int i=0;i<sensor.landmark_list_gb.size();i++){

    lnd_sample_v pose;

    pose.lnd_read = sensor.landmark_list_gb.back()
    pose.x_robot    = x;
    pose.y_robot    = y;
    pose.time_stamp   = time_stamp;
    lnd_sample_v.push_back(pose);

  }
}

  void printData(){

    for (int i=0;i<sensor.landmark_list_gb.size();i++){


      std::cout << lnd_sample_v.front().time_stamp << " " << lnd_sample_v.front().x_robot << " " <<lnd_sample_v.front().y_robot 
                << " " << lnd_sample_v.front().lnd_read.x << lnd_sample_v.front().lnd_read.y  << " " << lnd_sample_v.front().lnd_read.id
                << std::endl; 

                lnd_sample_v.pop_front();
    }
  }
};*/


///////////////////////////////////////////////////////////////////////////////
                                // ESTIMATOR //
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

  ros::init(argc, argv, "zekf");
  ros::NodeHandle n;

  ros::Subscriber actu_flag = n.subscribe("flag", 1000, FlagCallback);

  n.getParam("dt", dt);
  dt = 0.005;
  ros::Rate loop_rate(200);

  ros::service::waitForService("LoadMatrices", -1);

  ObserverLPV_SLAM observer;
  GetActuData actuators;
  GetSensorData sensors;
  ComputeError error_manager;
  GetGroundTruthData truth;
  GetMarkers markers;

  int i_it = 0;

  std::cout << "starting code read_SLAM" << std::endl;
  markers.GetMarkersStart();

  while (ros::ok())
  {

    if (1){

				observer.x      = truth.x;
				observer.y      = truth.y;
				observer.yaw    = wrap(truth.yaw);
				observer.vx     = truth.vx;
				observer.vy     = truth.vy;
				observer.psiDot = truth.psiDot;

        g_x = truth.x;
        g_y = truth.y;

        observer.estimateState(sensors, actuators, &markers);

        /*std::cout << "Elapsed time in milliseconds : "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << " µs" << std::endl;*/
       // error_manager.compute_error(truth, observer);
        i_it++;


        /*ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                          <<" vy "<< truth.vy <<" yaw " << truth.yaw <<" psiDot " << truth.psiDot << "iteration " << i_it);*/

    }

    else if(!truth.x_hist.empty()){

        truth.x_hist.pop_front();
        truth.y_hist.pop_front();
        truth.yaw_hist.pop_front();
        truth.vx_hist.pop_front();
        truth.vy_hist.pop_front();
        truth.psiDot_hist.pop_front();

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  observer.publishData();

  return 0;
}
