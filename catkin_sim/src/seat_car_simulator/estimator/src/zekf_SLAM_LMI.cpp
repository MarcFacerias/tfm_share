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
#include <estimator/LandmarksInfo.h>
#include <estimator/DataInfo.h>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <random>
using namespace Eigen; //matrix calculus lib

/*

Implementation of both interval and lpv kalman filter, in order to swap between them adapt propertly the gain matrices

*/

///////////////////////////////////////////////////////////////////////////////
                                // GLOBAL //
///////////////////////////////////////////////////////////////////////////////

float dt; //iteration time
bool flag = 0; //starting flag
float x_r; // sensor x
float y_r; // sensor y
float yaw_r; //sensor yaw
float x_t = 0; // true x
float y_t = 0; // true y
float x_gaz = 0; // gazebo x
float y_gaz = 0; // gazebo y
float yaw_gaz = 0; // gazebo yaw
float v_r; // sensor vel
bool gazebo = 0; // enable or disable gazebo

float sns_x = 0;
float sns_y = 0;

// generate landmark sensor noise
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> disR(-0.075, 0.075);
std::uniform_real_distribution<> disA(-0.065, 0.05);

///////////////////////////////////////////////////////////////////////////////
                             // CALLBACK and AUX //
///////////////////////////////////////////////////////////////////////////////

// implementation of a pseudoinverse
template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
  Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
  Eigen::Index rank = svd.rank();
  Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
                0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
  tmp = svd.matrixU().leftCols(rank).adjoint();
  tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
  return svd.matrixV().leftCols(rank) * tmp;
 }

// Landmark reading
struct PolarPose{

	float range;
	float angle;
	uint id;

};

// wrap angle between -pi and pi
float wrap(double deltaPhase)
{
    if (deltaPhase>0)
        deltaPhase = fmod(deltaPhase+M_PI, 2.0*M_PI)-M_PI;
    else
        deltaPhase = fmod(deltaPhase-M_PI, 2.0*M_PI)+M_PI;

    return deltaPhase;
}

// pose of a landmark + tubes(stored)
struct GlobalPose{

	float x;
	float y;
  float max_x;
  float max_y;
  float min_x;
  float min_y;
	uint id;

};

// x,y to polar coordinates
void Polar(float x, float y, PolarPose *pose){

	/*pose -> range     = sqrt(x*x + y*y) + disR(gen);
	pose -> angle     = atan2(y, x) + disA(gen);*/

  pose -> range     = sqrt(x*x + y*y);
  pose -> angle     = atan2(y, x);

}

void Cart(PolarPose *PolPose, GlobalPose *CartPose){

  CartPose -> x     = PolPose -> range * sin(PolPose -> angle);
  CartPose -> y     = PolPose -> range * cos(PolPose -> angle);

}


////////////////////////////classes////////////////////////////
// Landmark callback class
class GetMarkers{

  private:

    ros::NodeHandle n;
    ros::Subscriber marker_sub;

  public:

  	std::deque<PolarPose> landmark_list;

    void GetMarkersStart(){

      marker_sub = n.subscribe("/Corrected_Pose_s", 1, &GetMarkers::GetMarkersCallback, this);

    }

    void GetMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
    {

      //std::cout << "hi" << msg.markers.size() << std::endl;

    	for (int i = 0; i < msg.markers.size(); i++){

    		PolarPose pose;
        pose.id = msg.markers[i].id;

        if (gazebo){

          // simu with gazebo

          Polar(msg.markers[i].pose.pose.position.x, msg.markers[i].pose.pose.position.y, &pose);

          x_gaz = msg.markers[i].pose.pose.orientation.x;
          y_gaz = msg.markers[i].pose.pose.orientation.y;
          yaw_gaz = msg.markers[i].pose.pose.orientation.z;

        }

    		else{
          // simu without gazebo
          float true_xlm;
          float true_ylm;

          switch (pose.id){

            case 1:
              true_xlm   = 2.5;
              true_ylm   = 1.5;
              pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
              pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_r) + disA(gen);
              break;

            case 2:
              true_xlm   = 2.5;
              true_ylm   = 4.5;
              pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
              pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_r)+ disA(gen);
              break;
            case 3:
              true_xlm   = -0.5;
              true_ylm   = 6;
              pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
              pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_r)+ disA(gen);
              break;
            case 4:
              true_xlm   = -3;
              true_ylm   = 1;
              pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
              pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_r)+ disA(gen);
              break;
            case 5:
              true_xlm   = -3;
              true_ylm   = 3.5;
              pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
              pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_r)+ disA(gen);
              break;
            case 6:
              true_xlm   = -3;
              true_ylm   = 6;
              pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
              pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_r)+ disA(gen);
              break;
            case 7:
              true_xlm   = 0;
              true_ylm   = 1;
              pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
              pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_r)+ disA(gen);
              break;
            default:
            break;
          }

        }


        // if v>1 -> start estimation
        if (v_r > 1) {

         /*std::cout << msg.markers[i].id << " " << msg.markers[i].pose.pose.position.x << " " <<
                   msg.markers[i].pose.pose.position.y << " "  << x_t << " " << y_t << " "
                   << yaw_r << " " << x_gaz << " " << y_gaz << " " << yaw_gaz << std::endl;

         /*std::cout << msg.markers[i].id << " " << pose.range << " " <<
                   pose.angle << " "  << x_t << " " << y_t << " "
                   << yaw_r << " " << x_gaz << " " << y_gaz << " " << yaw_gaz << std::endl;*/

         landmark_list.push_back(pose);
       }
     }
  	}
};

// class collecting simulated sensor data
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

      pose_sub = n.subscribe("/sensorStates_s", 1, &GetSensorData::GetPoseCallback, this);

    }

    void GetPoseCallback(const lpv_mpc::simulatorStates& msg)
    {

        x       = msg.x;
        y       = msg.y;
        yaw     = msg.psi;
        vx      = msg.vx;
        psiDot  = msg.psiDot;

    }

};

// class collecting ground truth data
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

      groundTruth_sub = n.subscribe("/vehicle_state_s", 1, &GetGroundTruthData::GetGroundTruthDataCallback, this);

    }

    void GetGroundTruthDataCallback(const l4vehicle_msgs::VehicleState& msg)
    {

        x       = msg.x;
        y       = msg.y;
        yaw     = (msg.heading);
        vx      = msg.longitudinal_velocity;
        vy      = msg.lateral_velocity;
        psiDot  = msg.angular_velocity;

    }

};

// class collecting actuators data
class GetActuData{

  public:

    float a  = 0.0;
    float steer = 0.0;
    ros::Subscriber actu_sub;
    ros::NodeHandle n;

    GetActuData(){

      actu_sub = n.subscribe("/ecu_s", 1, &GetActuData::GetActuCallback, this);

    }

    private:

    void GetActuCallback(const lpv_mpc::ECU& msg)
    {

        a     = msg.motor;
        steer = msg.servo;

    }

};
// class listeing the starting flag
void FlagCallback(const std_msgs::Bool& msg){

  flag = msg.data;

}


/////////////////////////functions//////////////////////////

/*

Funtions related with the dimensionality reduction, implemented in matlab following Combastel papers

*/

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

MatrixXf reduction(MatrixXf Rxio, int n_dim){

	int n = Rxio.rows();
	int p = Rxio.cols();

	if (n_dim < n){

        ROS_ERROR_STREAM("Unable to perform reduction, actual dimension smaller than desired dimension");
        return Rxio;

	}

	if (p <= n_dim){

        ROS_ERROR_STREAM("Unable to perform reduction, complexity limit not achieved");
        return Rxio;

	}

	VectorXf NCZ = VectorXf::Zero(p);

	for (int i = 0; i < p; i++ ){

		for (int j = 0; j < n; j++){

			NCZ(i) +=  Rxio(j,i) * Rxio(j,i);

		}

	}

	int nvr = p - n_dim + n -1;

	VectorXf I      = sort_indexes(NCZ);
	VectorXf idx_s1 = I(seq(0,nvr));
	VectorXf idx_s2 = I(seq(nvr+1,p-1));

	MatrixXf Out;


	if (idx_s2.size() != 0){

		Out.resize(n,idx_s2.size());

		for (int i = 0; i < idx_s2.size(); i++){

			Out.col(i) = Rxio.col(idx_s2(i));

		}

	}

	if (idx_s1.size() != 0){

		MatrixXf Ms;
		Ms.resize(n,idx_s1.size());

		for (int i = 0; i < idx_s1.size(); i++){

			Ms.col(i)  = Rxio.col(idx_s1(i));


		}

		Ms = EnvBox(Ms);

		if (Out.cols() > 0){

			Out.conservativeResize(n,Out.cols() + idx_s1.size());

			Out(all, seq(Out.cols() - idx_s2.size(),Out.cols() )) = Ms;

		}
		else {

			Out.conservativeResize(n, idx_s1.size());
			Out = Ms;

		}

	}


	return Out;

}
///////////////////////////////////////////////////////////////////////////////
                                // ESTIMATOR //
///////////////////////////////////////////////////////////////////////////////

// class of the main observer
class ObserverLPV_SLAM{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    ros::Publisher landmark_pub = n.advertise<estimator::LandmarksInfo>("LandmarkData", 1);


    // model dimension variables
    float n_states_din  = 3;
    float n_outputs_din = 2;
    float n_control_din = 2;

    // model dimension variables
    float n_states_cin  = 5;
    float n_outputs_cin = 5;
    float n_control_cin = 3;

    //general usage variables
    // MatrixXf::Identity(8, 8);


    //vehicle variables
    float lf;
    float lr;
    float m;
    float I;
    float Cf;
    float Cr;
    float mu;
    float et = 0;

    //matrices dinamic
    MatrixXf A_din;
    MatrixXf B_din;
    MatrixXf C_din;
    MatrixXf Aln_din;
    MatrixXf Bln_din;
    MatrixXf L_din;
    MatrixXf Rxio_din = MatrixXf::Zero(3,3);
    MatrixXf Ew_din;
    MatrixXf Ev_din;
    MatrixXf Ew_diag_din;
    MatrixXf Ev_diag_din;
    MatrixXf Q_din;
    MatrixXf R_din;

    //matrices kinematic
    MatrixXf A_cin;
    MatrixXf B_cin;
    MatrixXf C_cin;
    MatrixXf Aln_cin;
    MatrixXf Bln_cin;
    MatrixXf L_cin;
    MatrixXf Rxio_cin = MatrixXf::Zero(5,5);
    MatrixXf Ew_cin;
    MatrixXf Ev_cin;
    MatrixXf Ew_diag_cin;
    MatrixXf Ev_diag_cin;
    MatrixXf Q_cin;
    MatrixXf R_cin;

    //Vectors
    VectorXf u_cin;
    VectorXf u_din;


    //DATA from matlab
    std::vector<MatrixXf> Llmi_cin;
    std::vector<MatrixXf> Llmi_din;
    std::vector<std::vector<float>> sched_vars_cin;
    std::vector<std::vector<float>> sched_vars_din;
    std::list <std::vector<float>> upper_limits;
    std::list <std::vector<float>> lower_limits;


    //DATA from landmarks
    std::deque<GlobalPose> lm_data;
    std::deque<MatrixXf>   R_lm_data;
    std::deque<MatrixXf>   P_lm_data;

    //estimated states initialization
    float x      = 0.02;
    float y      = 0.0;
    float vx     = 0.75;
    float vy     = 0.0;
    float yaw    = 0.0;
    float psiDot = 0.0;
    float lambda = 1.0012;
    VectorXf states_est_cin;
    VectorXf states_est_din;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;
    std::list <std::vector<float>> est_error_hist;
    estimator::LandmarksInfo lnd_list_msg;


  void estimateState(GetSensorData sensor, GetActuData ecu, GetMarkers *landmarks){
    // estimate the state of the system
    std::cout << "start Dyn estimate" << '\n';
    VectorXf y_meas_cin;
    VectorXf y_meas_din;
    std::cout << "start Dyn estimate" << '\n';
    u_din(0)      = ecu.steer;
    u_din(1)      = ecu.a;
    bool first_est = 1;
    bool lnd_detected = 0;
    std::cout << "start Dyn estimate" << '\n';
    //update states
    vx      = states_est_din(0);
    vy      = states_est_din(1);
    psiDot  = states_est_din(2);
    std::cout << "start Dyn estimate" << '\n';
    x       = states_est_cin(0);
    y       = states_est_cin(1);
    yaw     = states_est_cin(2);

    // Dinamic states
      std::cout << "start Dyn estimate -> Error below" << '\n';
      AB_computation_din(u_din(0),vx ,vy, first_est); //update LPV matrices
      std::cout << "start Dyn estimate -> Error below" << '\n';
      L_computation_din(u_din(0), vx, vy); // compute L gain with matlab
      std::cout << "Dyn estimate 1" << '\n';
      // Implementation of the iterative kalman filter
      MatrixXf x_pred = Aln_din *states_est_din+ Bln_din*u_din;
      std::cout << "Dyn estimate 2" << '\n';

      VectorXf error = y_meas_din - C_din*x_pred;
      std::cout << "Dyn estimate 3" << '\n';
      states_est_din = x_pred + L_din*error;

      std::cout << "Dyn estimate" << '\n';
    // // Cinematic states
    // // asuming we detect landmarks
  	if (!landmarks->landmark_list.empty())
  	{

      // if is the first landmatk that we deal with (avoid entering empty vectors)
      if (lm_data.empty()){

        GlobalPose newPose;
        newPose.id = landmarks->landmark_list.back().id;
        newPose.x  = x + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
        newPose.y  = y + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);

        lm_data.push_back(newPose);
        landmarks->landmark_list.pop_back();
      }

      // once the vector is not empty, start dealing with all the landmarks detected in current iteration
  		while (!landmarks->landmark_list.empty() && (lnd_detected == 0)){

  			int it;
  			for (it = 0; it < lm_data.size(); it++){

  				if (lm_data[it].id == landmarks->landmark_list.back().id){

  					lnd_detected = 1;
  					break;

  				}
  			}

  		// if the observer landmark is not asociated with an observer matlab, add it and end while
  		if (!lnd_detected){

  			GlobalPose newPose;
  			newPose.id = landmarks->landmark_list.back().id;
  			newPose.x  = x + landmarks->landmark_list.back().range*cos(landmarks->landmark_list.back().angle + yaw);
  			newPose.y  = y + landmarks->landmark_list.back().range*sin(landmarks->landmark_list.back().angle + yaw);

  			lm_data.push_back(newPose);

	      landmarks->landmark_list.pop_back();
	  	  break;

  		}

      // update landmark info
  		states_est_cin(6) = lm_data[it].x;
  		states_est_cin(7) = lm_data[it].y;

      // resize matrices, probably redundant
  		if (y_meas_cin.size() != n_outputs_cin){

  			y_meas_cin.conservativeResize( n_outputs_cin);

  		}

      if (y_meas_din.size() != n_outputs_din){

  			y_meas_din.conservativeResize( n_outputs_din);

  		}

      // update y vector
  		y_meas_din(0) = sensor.vx;
  		y_meas_din(1) = sensor.psiDot;

  		y_meas_cin(0) = sensor.x;
  		y_meas_cin(1) = sensor.y;
  		y_meas_cin(2) = sensor.yaw;
  		y_meas_cin(3) = landmarks->landmark_list.back().range;
  		y_meas_cin(4) = landmarks->landmark_list.back().angle;

      int newPose_id = landmarks->landmark_list.back().id;

	    AB_computation_cin(y_meas_cin(2) ,first_est); //update LPV matrices
	    L_computation_cin(y_meas_cin(2)); // compute L gain with matlab

      // Implementation of the iterative kalman filter
      MatrixXf x_pred = Aln_cin *states_est_cin+ Bln_cin*u_cin;

      VectorXf error = y_meas_cin - C_cin*x_pred;

      states_est_cin = x_pred + L_cin*error;

      //Store center landmark data
      lm_data[it].x = states_est_cin(3);
      lm_data[it].y = states_est_cin(4);

      //Remove observed landmark

      landmarks->landmark_list.pop_back();
      storeLandmarkData(lm_data[it].x,lm_data[it].y,lm_data[it].id,lm_data[it].max_x,lm_data[it].min_x,lm_data[it].max_y,lm_data[it].min_y);
      lnd_detected = 0;
      first_est = 0;
  	 }
  	}

    // estimate whenever no landmark is detected
  	else if(first_est){

        // update y vector
        y_meas_cin.conservativeResize(n_outputs_cin - 2);
        y_meas_cin(0) = sensor.x;
        y_meas_cin(1) = sensor.y;
        y_meas_cin(2) = sensor.yaw;

        // update matrices
        AB_computation_cin(y_meas_cin(2),1);

        L_computation_cin(y_meas_cin(2)); // compute L gain with matlab


        // compute KF
        MatrixXf x_pred = Aln_cin(seq(0,5),seq(0,5)) *states_est_cin(seq(0,5)) + Bln_cin(seq(0,5),all)*u_cin;

        VectorXf error = y_meas_cin - C_cin(seq(0,4),seq(0,5))*x_pred;

        states_est_cin(seq(0,5)) = x_pred + L_cin(seq(0,2),seq(0,2))*error;

  	}

        // Store info
        x_est_hist.push_back(x);
        y_est_hist.push_back(y);
        yaw_est_hist.push_back(yaw);
        vx_est_hist.push_back(vx);
        x_r = x;
        y_r = y;
        vy_est_hist.push_back(vy);
        psiDot_est_hist.push_back(psiDot);

        // MatrixXf LimitMatrix = EnvBox(Rxio);
        //
        // std::vector<float> aux_vec_max;
        // std::vector<float> aux_vec_min;
        //
        // for (int i = 0; i < 6; i++){
        //
        //  aux_vec_max.push_back(states_est(i) + fabs(LimitMatrix(i,i)));
        //  aux_vec_min.push_back(states_est(i) - fabs(LimitMatrix(i,i)));
        //
        // }
        //
        // //Store interval limits
        // upper_limits.push_back(aux_vec_max);
        // lower_limits.push_back(aux_vec_min);


      }

    // Initialization function called when created a class
    ObserverLPV_SLAM(){

      //kinematic

      A_cin.resize(n_states_cin,n_states_cin);
      B_cin.resize(n_states_cin,n_control_cin);
      Aln_cin.resize(n_states_cin,n_states_cin);
      Bln_cin.resize(n_states_cin,n_control_cin);
      C_cin.resize(n_outputs_cin,n_states_cin);
      L_cin.resize(n_states_cin,n_outputs_cin);

      // Ew_cin.resize(n_states_cin,n_states_cin);
      // Ev_cin.resize(n_outputs_cin,n_states_cin);
      // Ew_diag_cin.resize(n_states_cin,n_states_cin);
      // Ev_diag_cin.resize(n_outputs_cin,n_outputs_cin);

      u_cin.resize(n_control_cin);
      states_est_cin.resize(n_states_cin);

      Q_cin.resize(n_states_cin,n_states_cin);
      R_cin.resize(n_outputs_cin,n_outputs_cin);

      //dinamic

      A_din.resize(n_states_din,n_states_din);
      B_din.resize(n_states_din,n_control_din);
      Aln_din.resize(n_states_din,n_states_din);
      Bln_din.resize(n_states_din,n_control_din);
      C_din.resize(n_outputs_din,n_states_din);
      L_din.resize(n_states_din,n_outputs_din);

      // Ew_din.resize(n_states_din);
      // Ev_din.resize(n_outputs_din);
      // Ew_diag_din.resize(n_states_din,n_states_din);
      // Ev_diag_din.resize(n_outputs_din,n_outputs_din);

      u_din.resize(n_control_din);
      states_est_din.resize(n_states_din);

      Q_din.resize(n_states_din,n_states_din);
      R_din.resize(n_outputs_din,n_outputs_din);

      // load launch information
      n.getParam("lf",lf);
      n.getParam("lr", lr);
      n.getParam("m", m);
      n.getParam("Iz",I);
      n.getParam("Cf",Cf);
      n.getParam("Cr",Cr);
      n.getParam("mu",mu);

      // initialize matrices
      C_din << 1, 0, 0,
               0, 0, 1;

      C_cin << 1, 0, 0, 0, 0,
               0, 1, 0, 0, 0,
               0, 0, 1, 0, 0,
               0, 0, 0, 0, 0,
               0, 0, 0, 0, 0;

      B_din << 0, 1,
               0, 0,
               0, 0;
      std::cout << n_states_cin << n_control_cin;
      B_cin << 0, 1, 0,
               0, 0, 0,
               0, 0, 0,
               0, 0, 0,
               0, 0, 0;

      std::cout << "init";
      Q_din << 0.0006, 0,      0,
               0,      0.0009, 0,
               0,      0,      0.0627;

      R_din << 0.003, 0,
               0,      0.0049;
      std::cout << "init";
      Q_cin << 0.0006, 0,      0,      0,      0,
               0,      0.0009, 0,      0,      0,
               0,      0,      0.0627, 0,      0,
               0,      0,      0,      0.0001, 0,
               0,      0,      0,      0,      0.0001;


      R_cin << 0.003,  0,      0,      0,      0,
               0,      0.0049, 0,      0,      0,
               0,      0,      0.0049, 0,      0,
               0,      0,      0,      0.0049, 0,
               0,      0,      0,      0,      0.0303;

      u_din << 0,0;
      u_cin << 0,0,0;
      /*Ew <<0.01252 , 0.0152, 0.1252 ,0.004 , 0.004 , 0.008 , 0.001 , 0.001;
      Ev << 0.08 , 0.035,  0.035 ,0.035 , 0.087 ,0.01 , 0.0896;
      Ew_diag = MatrixXf::Zero(n_states,n_states);
    Ev_diag = MatrixXf::Zero(n_outputs,n_outputs);



      for (int i=0; i<n_states; i++){

        Ew_diag(i,i) = Ew(i);

      }

      for (int i=0; i<n_outputs; i++){

        Ev_diag(i,i) = Ev(i);

      }*/

      states_est_din << vx, vy, psiDot;
      states_est_cin << x, y, yaw, 0, 0;

      // Load gain matrices, if using the LMI
      LoadMatrices_din("LMI_din");
      LoadMatrices_cin("LMI_cin");

    }

    void storeLandmarkData(float x, float y, float id, float max_x, float max_y, float min_x, float min_y){

      // Function used to store landmark data

      estimator::LandmarkInfo lnd_msg;

        lnd_msg.x.push_back(x);
        lnd_msg.y.push_back(y);
        lnd_msg.id.push_back(id);
        /*lnd_msg.max_x.push_back(max_x);
        lnd_msg.max_y.push_back(max_y);
        lnd_msg.min_x.push_back(min_x);
        lnd_msg.min_y.push_back(min_y);*/

        //std::cout << max_y << " " << max_x << " " << min_x << " " << min_y << std::endl;

      lnd_list_msg.lnd_list.push_back(lnd_msg);

    }

    private:

    // Load LMI matrices from matlab
    void LoadMatrices_cin(std::string file){
      estimator::lmi_data srv;
      srv.request.est_id = "LMI_cin";
      client.call(srv);

      ROS_ERROR_STREAM("Loading" << srv.response.L.size() << " matrices ");
      Llmi_cin.resize(srv.response.L.size());

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi_cin[i].resize(n_states_cin,n_outputs_cin);

        for (int rows = 0; rows < n_states_cin; rows++){

          for (int cols = 0; cols < n_outputs_din; cols++){

            Llmi_cin[i](rows,cols) = srv.response.L[i].gains[rows*(n_outputs_cin) + cols];

          }
        }
      }
      std::cout << "end Llmi Gain" << std::endl;
      //fill sched_vars

      sched_vars_cin.resize(srv.response.limits.max.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars_cin[i].push_back(srv.response.limits.min[i]);
        sched_vars_cin[i].push_back(srv.response.limits.max[i]);

      }
    }

    // Load LMI matrices from matlab
    void LoadMatrices_din(std::string file){
      estimator::lmi_data srv;
      srv.request.est_id = "LMI_din";
      client.call(srv);

      ROS_ERROR_STREAM("Loading" << srv.response.L.size() << " matrices ");
      Llmi_din.resize(srv.response.L.size());

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi_din[i].resize(n_states_din,n_outputs_din);

        for (int rows = 0; rows < n_states_din; rows++){

          for (int cols = 0; cols < n_outputs_din; cols++){

            Llmi_din[i](rows,cols) = srv.response.L[i].gains[rows*(n_outputs_din) + cols];

          }
        }
      }
      std::cout << "end Llmi Gain" << std::endl;
      //fill sched_vars

      sched_vars_din.resize(srv.response.limits.max.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars_din[i].push_back(srv.response.limits.min[i]);
        sched_vars_din[i].push_back(srv.response.limits.max[i]);

      }
    }


    void AB_computation_cin(float yaw , bool first_est){

      if (first_est) {

  	    //Update B
	      B_cin(0,0) = cos(yaw);
	      B_cin(1,0) = -sin(yaw);

	      B_cin(0,1) = sin(yaw);
	      B_cin(1,1) = cos(yaw);

  	    B_cin(2,2) = 1;


      }

      else{

  	    //Update B
	      B_cin(0,0) = 0;
	      B_cin(1,0) = 0;

	      B_cin(0,1) = 0;
	      B_cin(1,1) = 0;

  	    B_cin(2,2) = 0;

      }

      Aln_cin = MatrixXf::Identity(n_states_cin,n_states_cin) + (A_cin*dt);
      Bln_cin = B_cin*dt;

    }

    void AB_computation_din(float steer,float vx,float vy, bool first_est){

      //update B
      B_din(0,0) = -sin(steer) * Cf/m;
      B_din(1,0) = (cos(steer) * Cf) / m;
      B_din(2,0) = (lf * Cf * cos(steer)) / I;
      B_din(1,1) = cos(steer);
      B_din(2,1) = sin(steer);


      if (first_est) {

  	    //Update A
	      A_din(0,0) =  -mu;
	      A_din(0,1) = (sin(steer) * Cf) / (m*vx);
	      A_din(0,2) = (sin(steer) * Cf * lf) / (m*vx) + vy;

	      A_din(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
	      A_din(1,2) = -(lf * Cf * cos(steer) - lr * Cr) / (m*vx) - vx;

	      A_din(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
	      A_din(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);

      }

      else{

        A_din = MatrixXf::Zero(n_states_din,n_states_din);
        B_din = MatrixXf::Zero(n_states_din,n_outputs_din);

      }

      Aln_din = MatrixXf::Identity(n_states_din,n_states_din) + (A_din*dt);
      Bln_din = B_din*dt;

    }


    // LPV interpolation of the gains with landmark
    void L_computation_cin(float theta_lm){

		float M_Stheta_lm       = (sched_vars_cin[0][1] - sin(theta_lm)) / (sched_vars_cin[0][1] - sched_vars_cin[0][0]);
		float M_Ctheta_lm       = (sched_vars_cin[1][1] - cos(theta_lm)) / (sched_vars_cin[1][1] - sched_vars_cin[1][0]);

		VectorXf mu_sch;
		mu_sch.resize(16);

    mu_sch[0]               = M_Stheta_lm         * M_Ctheta_lm      * M_Stheta_lm      *  M_Ctheta_lm;
    mu_sch[1]               = M_Stheta_lm         * M_Ctheta_lm      * M_Stheta_lm      *  (1-M_Ctheta_lm);
    mu_sch[2]               = M_Stheta_lm         * M_Ctheta_lm      * (1-M_Stheta_lm)  *  M_Ctheta_lm;
    mu_sch[3]               = M_Stheta_lm         * M_Ctheta_lm      * (1-M_Stheta_lm)  *  (1-M_Ctheta_lm);
    mu_sch[4]               = M_Stheta_lm         * (1-M_Ctheta_lm)  * M_Stheta_lm      *  M_Ctheta_lm;
    mu_sch[5]               = M_Stheta_lm         * (1-M_Ctheta_lm)  * M_Stheta_lm      *  (1-M_Ctheta_lm);
    mu_sch[6]               = M_Stheta_lm         * (1-M_Ctheta_lm)  * (1-M_Stheta_lm)  *  M_Ctheta_lm;
    mu_sch[7]               = M_Stheta_lm         * (1-M_Ctheta_lm)  * (1-M_Stheta_lm)  *  (1-M_Ctheta_lm);

    mu_sch[8]               = (1-M_Stheta_lm)     * M_Ctheta_lm      * M_Stheta_lm      *  M_Ctheta_lm;
    mu_sch[9]               = (1-M_Stheta_lm)     * M_Ctheta_lm      * M_Stheta_lm      *  (1-M_Ctheta_lm);
    mu_sch[10]              = (1-M_Stheta_lm)     * M_Ctheta_lm      * (1-M_Stheta_lm)  *  M_Ctheta_lm;
    mu_sch[11]              = (1-M_Stheta_lm)     * M_Ctheta_lm      * (1-M_Stheta_lm)  *  (1-M_Ctheta_lm);
    mu_sch[12]              = (1-M_Stheta_lm)     * (1-M_Ctheta_lm)  * M_Stheta_lm      *  M_Ctheta_lm;
    mu_sch[13]              = (1-M_Stheta_lm)     * (1-M_Ctheta_lm)  * M_Stheta_lm      *  (1-M_Ctheta_lm);
    mu_sch[14]              = (1-M_Stheta_lm)     * (1-M_Ctheta_lm)  * (1-M_Stheta_lm)  *  M_Ctheta_lm;
    mu_sch[15]              = (1-M_Stheta_lm)     * (1-M_Ctheta_lm)  * (1-M_Stheta_lm)  *  (1-M_Ctheta_lm);

		MatrixXf result = MatrixXf::Zero(n_states_cin,n_states_cin);

		for (int i = 0; i < 64; i++){

		  result += mu_sch[i] * Llmi_cin[i];

		}

		L_cin = result;
    }

    // LPV interpolation of the gains without landmark

    void L_computation_din(float steer, float vx, float vy){


      float M_vx_despl_min    = (sched_vars_din[0][1] - vx)    / (sched_vars_din[0][1] - sched_vars_din[0][0]);
      float M_vy_despl_min    = (sched_vars_din[1][1] - vy)    / (sched_vars_din[1][1] - sched_vars_din[1][0]);
      float M_steer_min       = (sched_vars_din[3][1] - steer) / (sched_vars_din[3][1] - sched_vars_din[3][0]);

      VectorXf mu_sch;
      mu_sch.resize(8);

      mu_sch[0]               = M_vx_despl_min      * M_steer_min      *  M_vy_despl_min;
      mu_sch[1]               = M_vx_despl_min      * M_steer_min      *  (1-M_vy_despl_min);
      mu_sch[2]               = M_vx_despl_min      * (1-M_steer_min)  *  M_vy_despl_min;
      mu_sch[3]               = M_vx_despl_min      * (1-M_steer_min)  *  (1-M_vy_despl_min);

      mu_sch[4]               = (1-M_vx_despl_min)  * M_steer_min      *  M_vy_despl_min;
      mu_sch[5]               = (1-M_vx_despl_min)  * M_steer_min      *  (1-M_vy_despl_min);
      mu_sch[6]               = (1-M_vx_despl_min)  * (1-M_steer_min)  *  M_vy_despl_min;
      mu_sch[7]               = (1-M_vx_despl_min)  * (1-M_steer_min)  *  (1-M_vy_despl_min);


      MatrixXf result = MatrixXf::Zero(n_states_din,n_outputs_din);

      for (int i = 0; i < 8; i++){

          result += mu_sch[i] * Llmi_din[i];

      }

      L_din = result*dt;
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

    error_pub = n.advertise<estimator::ErrorsInfo>("errors", 1);
    data_pub = n.advertise<estimator::DataInfo>("data", 1);

  }

    ///////////////////////////////////////////////////////////////////
   /////////////ERROR FUNCTIONS, may be computed in matlab////////////
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
    float numel = 0;/* message_hi2 */


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

    std::cout << truth.x_hist.size() << " " << estimator.x_est_hist.size() << " " << estimator.upper_limits.size();

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

      aux.x       = estimator.upper_limits.front()[3];
      aux.y       = estimator.upper_limits.front()[4];
      aux.psi     = estimator.upper_limits.front()[5];
      aux.vx      = estimator.upper_limits.front()[0];
      aux.vy      = estimator.upper_limits.front()[1];
      aux.psiDot  = estimator.upper_limits.front()[2];

      estimator.upper_limits.pop_front();

      msg_data.max_limits.push_back(aux);

      aux.x       = estimator.lower_limits.front()[3];
      aux.y       = estimator.lower_limits.front()[4];
      aux.psi     = estimator.lower_limits.front()[5];
      aux.vx      = estimator.lower_limits.front()[0];
      aux.vy      = estimator.lower_limits.front()[1];
      aux.psiDot  = estimator.lower_limits.front()[2];

      estimator.lower_limits.pop_front();

      msg_data.min_limits.push_back(aux);

    }

    error_pub.publish(msg);
    data_pub.publish(msg_data);

    std::cout << "msg published" <<std::endl;

  }
};


///////////////////////////////////////////////////////////////////////////////
                                // ESTIMATOR //
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

  ros::init(argc, argv, "zekf");
  ros::NodeHandle n;

  ros::Subscriber actu_flag = n.subscribe("SyncFlag", 1, FlagCallback);
  //ros::Publisher est_pub = n.advertise<lpv_mpc::pos_info>("/pos_info", 1); //Close Estimation Loop

  n.getParam("dt", dt);
  std::cout << "dt: " << dt;
  ros::Rate loop_rate(200);

  ros::service::waitForService("LoadMatrices", -1);

  ObserverLPV_SLAM observer;

  GetActuData actuators;

  GetSensorData sensors;
  ComputeError error_manager;
  GetGroundTruthData truth;
  GetMarkers markers;

  int i_it = 0;

  std::cout << "end config" << std::endl;

  auto start = std::chrono::steady_clock::now();

  while (ros::ok() && i_it < 3000)
  {
    std::cout << "actuators"<< actuators.a << " " << actuators.steer << " " << truth.vx << std::endl;
    if (flag && (actuators.a != 0) && (actuators.steer != 0) && (truth.vx > 1)){
        // std::cout << "/* message_hi */" << std::endl;
        if (i_it == 0){

          // initialize estimator matrices
          start = std::chrono::steady_clock::now();
          // std::cout << "/* message_hi2 */" << std::endl;

          markers.GetMarkersStart();

          observer.states_est_cin(0)      = truth.x;
          observer.states_est_cin(1)      = truth.y;
          observer.states_est_cin(2)      = truth.yaw;

          observer.states_est_din(0)      = truth.vx;
          observer.states_est_din(1)      = truth.vy;
          observer.states_est_din(2)      = truth.psiDot;

        }

        x_t   = truth.x;
		    y_t   = truth.y;
        yaw_r = truth.yaw;
        v_r   = truth.vx;

        observer.estimateState(sensors, actuators, &markers);

        // Close Estimation Loop
        /*lpv_mpc::pos_info est_msg;

        est_msg.x                   = observer.x;
        est_msg.y                   = observer.y;
        est_msg.psi                 = observer.yaw;
        est_msg.v                   = sqrt(observer.vx*truth.vx + observer.vy*truth.vy);
        est_msg.v_x                 = observer.vx;
        est_msg.v_y                 = observer.vy;
        est_msg.psiDot              = observer.psiDot;

        est_pub.publish(est_msg);*/

        truth.x_hist.push_back(truth.x);
        truth.y_hist.push_back(truth.y);
        truth.yaw_hist.push_back(truth.yaw);
        truth.vx_hist.push_back(truth.vx);
        truth.vy_hist.push_back(truth.vy);
        truth.psiDot_hist.push_back(truth.psiDot);

        //error_manager.compute_error(truth, observer);
        i_it++;


	    ROS_ERROR_STREAM("update finished: x " << observer.x <<" y "<< observer.y <<" yaw " << observer.yaw <<" vx " << observer.vx
	                <<" vy "<< observer.vy <<" psiDot " << observer.psiDot );


      ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                          <<" vy "<< truth.vy <<" psiDot " << truth.psiDot << "iteration " << i_it);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ROS_ERROR_STREAM("update finished: vx " << observer.vx <<" vy "<< observer.vy <<" psiDot " << observer.psiDot );


      ROS_ERROR_STREAM("Ground Truth: vx " << truth.vx <<" vy "<< truth.vy <<" psiDot " << truth.psiDot << "iteration " << i_it);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ROS_ERROR_STREAM("update finished: x " << observer.x <<" y "<< observer.y <<" yaw " << observer.yaw);


      ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw );
    }

    else if(!truth.x_hist.empty()){

      /*
        // Close Estimation Loop


        lpv_mpc::pos_info est_msg;

        est_msg.x                   = truth.x;
        est_msg.y                   = truth.y;
        est_msg.psi                 = truth.yaw;
        est_msg.v                   = sqrt(truth.vx*truth.vx + truth.vy*truth.vy);
        est_msg.v_x                 = truth.vx;
        est_msg.v_y                 = truth.vy;
        est_msg.psiDot              = truth.psiDot;

        est_pub.publish(est_msg);*/


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

  auto end = std::chrono::steady_clock::now();

	/*std::cout << "Elapsed time in milliseconds : "
	          << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
	          << " s" << std::endl;*/

  error_manager.save_error(truth, observer);
  observer.landmark_pub.publish(observer.lnd_list_msg);

  return 0;
}
