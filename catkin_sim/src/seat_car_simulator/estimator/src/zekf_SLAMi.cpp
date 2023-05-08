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
using namespace std;
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
float yaw_t; //sensor yaw
float x_t = 0; // true x
float y_t = 0; // true y
float x_gaz = 0; // gazebo x
float y_gaz = 0; // gazebo y
float yaw_gaz = 0; // gazebo yaw
float v_r; // sensor vel
bool gazebo = 1; // enable or disable gazebo

float sns_x = 0;
float sns_y = 0;

// generate landmark sensor noise
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> disR(-0.1, 0.1);
std::uniform_real_distribution<> disA(-0.1, 0.1);

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

struct CartPose{

	float x;
	float y;
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

void Polar(float x, float y, PolarPose *pose){

  pose -> range     = sqrt(x*x + y*y);
  pose -> angle     = atan2(y, x);

}

void Cart(PolarPose *PolPose, CartPose *CartPose){

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

  	deque<CartPose> landmark_list;
    deque<CartPose> landmark_hist;
    void GetMarkersStart(){

      marker_sub = n.subscribe("/Corrected_Pose_s", 1, &GetMarkers::GetMarkersCallback, this);

    }

    void GetMarkersCallback(const std_msgs::Bool& msg)
    {

      //cout << "hi" << msg.markers.size() << endl;

          // simu without gazebo
          float true_xlm;
          float true_ylm;

          for (int i = 1; i < 8; i++) {
            /* code */
            CartPose pose;
            pose.id = i;
            switch (pose.id){

                case 1:
                  true_xlm   = 2.5;
                  true_ylm   = 1.5;
                  // pose.range = sqrt((true_ylm-y_t)*(true_ylm-y_t) + (true_xlm-x_t)*(true_xlm-x_t)) + disR(gen);
                  // pose.angle = atan2(true_ylm-y_t,true_xlm-x_t)- wrap(yaw_t) + disA(gen);
                  break;

                case 2:
                  true_xlm   = 2.5;
                  true_ylm   = 4.5;
                  break;
                case 3:
                  true_xlm   = -0.5;
                  true_ylm   = 6;
                  break;
                case 4:
                  true_xlm   = -3;
                  true_ylm   = 1;
                  break;
                case 5:
                  true_xlm   = -3;
                  true_ylm   = 3.5;
                  break;
                case 6:
                  true_xlm   = -3;
                  true_ylm   = 6;
                  break;
                case 7:
                  true_xlm   = 0;
                  true_ylm   = 1;
                  break;
                default:
                break;
            }

          int dist = sqrt((true_xlm- x_t)*(true_xlm- x_t) + (true_ylm- y_t)*(true_ylm- y_t));

          if (dist < 1.5){
            pose.x  = - x_t *cos(yaw_t) - y_t *sin(yaw_t) + true_xlm*cos(yaw_t) + true_ylm*sin(yaw_t) + disR(gen);
            pose.y  =   x_t *sin(yaw_t) - y_t *cos(yaw_t) - true_xlm*sin(yaw_t) + true_ylm*cos(yaw_t) + disA(gen);
            // pose.x = x_t + true_xlm*cos(yaw_t) - true_ylm*sin(yaw_t); //+ disR(gen);
            // pose.y = y_t + true_xlm*sin(yaw_t) + true_ylm*cos(yaw_t); //+ disA(gen);

            /*cout << "Land Data: "<< true_xlm << " " << true_ylm << " " << pose.id << " " << pose.x << " " << pose.y << " "  << x_t << " " << y_t << " "
                      << yaw_t << endl;*/

            if (v_r > 1) {

             /*cout << msg.markers[i].id << " " << msg.markers[i].pose.pose.position.x << " " <<
                       msg.markers[i].pose.pose.position.y << " "  << x_t << " " << y_t << " "
                       << yaw_t << " " << x_gaz << " " << y_gaz << " " << yaw_gaz << endl;

             /*cout << msg.markers[i].id << " " << pose.range << " " <<
                       pose.angle << " "  << x_t << " " << y_t << " "
                       << yaw_t << " " << x_gaz << " " << y_gaz << " " << yaw_gaz << endl;*/

             landmark_list.push_back(pose);
             CartPose pose_old = pose;
             pose.x  = x_t + pose_old.x*cos(yaw_t) - pose_old.y*sin(yaw_t);
             pose.y  = y_t + pose_old.x*sin(yaw_t) + pose_old.y*cos(yaw_t);

             landmark_hist.push_back(pose);
           }
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

      pose_sub = n.subscribe("/sensorStates", 1, &GetSensorData::GetPoseCallback, this);

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

      groundTruth_sub = n.subscribe("/vehicle_state", 1, &GetGroundTruthData::GetGroundTruthDataCallback, this);

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

      actu_sub = n.subscribe("/ecu", 1, &GetActuData::GetActuCallback, this);

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
    ros::Publisher landmark_pub = n.advertise<estimator::LandmarksInfo>("LandmarkDataEKF", 1);


    // model dimension variables
    float n_states  = 8;
    float n_outputs = 7;
    float n_control = 2;

    //general usage variables
    MatrixXf eye8 = MatrixXf::Identity(8, 8);


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
    MatrixXf L_nlm;
    MatrixXf Rxio = MatrixXf::Zero(8,8);
    MatrixXf Ew_diag;
    MatrixXf Ev_diag;
    MatrixXf P;
    MatrixXf P_pred;
    MatrixXf Q;
    MatrixXf R;

    //Vectors
    VectorXf Ew;
    VectorXf Ev;
    VectorXf u;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<MatrixXf> Llmi_nlm;
    std::vector<std::vector<float>> sched_vars;
    std::vector<std::vector<float>> sched_vars_lm;
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
    VectorXf states_est;

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
    VectorXf y_meas;
    u(0)      = ecu.steer;
    u(1)      = ecu.a;
    bool first_est = 1;
    bool lnd_detected = 0;

    //update states
    vx      = states_est(0);
    vy      = states_est(1);
    psiDot  = states_est(2);
    x       = states_est(3);
    y       = states_est(4);
    yaw     = states_est(5);

    // asuming we detect landmarks
  	if (!landmarks->landmark_list.empty())
  	{

      // if is the first landmatk that we deal with (avoid entering empty vectors)
      if (lm_data.empty()){

        GlobalPose newPose;
        newPose.id = landmarks->landmark_list.back().id;
        newPose.x  = x + landmarks->landmark_list.back().x*cos(yaw) - landmarks->landmark_list.back().y*sin(yaw);
        newPose.y  = y + landmarks->landmark_list.back().x*sin(yaw) + landmarks->landmark_list.back().y*cos(yaw);

        lm_data.push_back(newPose);
        MatrixXf R_lm  = eye8;
        R_lm_data.push_back(R_lm);
        MatrixXf P_lm  = 0.01*MatrixXf::Identity(n_states,n_states);
        P_lm_data.push_back(P_lm);

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
            newPose.x  = x + landmarks->landmark_list.back().x*cos(yaw) - landmarks->landmark_list.back().y*sin(yaw);
            newPose.y  = y + landmarks->landmark_list.back().x*sin(yaw) + landmarks->landmark_list.back().y*cos(yaw);

  		    /*std::cout <<  landmarks->landmark_list.back().id << endl;
            std::cout <<  x + landmarks->landmark_list.back().x*cos(yaw) - landmarks->landmark_list.back().y*sin(yaw) << endl;
            std::cout <<  y + landmarks->landmark_list.back().x*sin(yaw) + landmarks->landmark_list.back().y*cos(yaw) << endl;

            float aux_x =  x_t + landmarks->landmark_list.back().x*cos(yaw_t) - landmarks->landmark_list.back().y*sin(yaw_t);
            float aux_y =  y_t + landmarks->landmark_list.back().x*sin(yaw_t) + landmarks->landmark_list.back().y*cos(yaw_t);
            std::cout <<  "Expected Landmark" << endl;
            std::cout <<  aux_x << endl;
            std::cout <<  aux_y << endl;
            AB_computation(u(0), first_est);

            VectorXf x_aux = VectorXf::Zero(8);
            x_aux(6) = aux_x;
            x_aux(7) = aux_y;
            cout << endl << C << endl;
            VectorXf est_sens = C*x_aux;
            cout << endl << endl << "est_sens" << endl;
            cout << est_sens << endl;
            cout << endl << endl << "Actual Readings" << endl;
            cout <<  landmarks->landmark_list.back().x << endl;
            cout <<  landmarks->landmark_list.back().y << endl << endl;*/

  			lm_data.push_back(newPose);

            MatrixXf R_lm  = MatrixXf::Zero(8,8);
            R_lm_data.push_back(R_lm);

            MatrixXf P_lm  = 0.01*MatrixXf::Identity(n_states,n_states);
            P_lm_data.push_back(P_lm);

	        landmarks->landmark_list.pop_back();
	  	    break;

  		}

      // resize matrices, probably redundant
	    if (Rxio.rows() < 8){

	        Rxio.conservativeResize(8,8);

	    }

      // update landmark info
  		states_est(6) = lm_data[it].x;
  		states_est(7) = lm_data[it].y;

      // resize matrices, probably redundant
  		if (y_meas.size() != n_outputs){

  			y_meas.conservativeResize(n_outputs);

  		}

      // update y vector
  	  y_meas(0) = sensor.vx;
  	  y_meas(1) = sensor.psiDot;
  	  y_meas(2) = sensor.x;
  	  y_meas(3) = sensor.y;
  	  y_meas(4) = sensor.yaw;
  	  y_meas(5) = landmarks->landmark_list.back().x;
  	  y_meas(6) = landmarks->landmark_list.back().y;

      int newPose_id = landmarks->landmark_list.back().id;

	  AB_computation(u(0), first_est); //update LPV matrices

      // Uptade matrix P (iterative version of KF)
      P(seq(6,7),all) = P_lm_data[it](seq(6,7),all);
      P(all,seq(6,7)) = P_lm_data[it](all,seq(6,7));

      P_pred = Aln * P* Aln.transpose() + Q;

      // Implementation of the iterative kalman filter
      MatrixXf x_pred = Aln *states_est+ Bln*u;

      VectorXf error = y_meas - C*x_pred;

      MatrixXf Z     = C*P_pred*C.transpose() + R;
      L              = P_pred*C.transpose()*pseudoInverse(Z);
      P              = P_pred - L * C * P_pred;

      states_est = x_pred + L*error;

     /* std::cout << endl << endl <<"Sys data" << std::endl;
      std::cout << endl << endl <<x_pred;
      std::cout << endl << endl <<C*x_pred;
      std::cout << endl << endl <<y_meas;
      std::cout << endl << endl <<lm_data[it].x;
      std::cout << endl << endl <<lm_data[it].y;
      std::cout << endl << endl <<lm_data[it].id;

      std::cout << endl << endl << Aln;
      std::cout << endl<< endl << Bln;
      std::cout << endl<< endl << C;
      std::cout << endl<< endl << L;
      std::cout << endl<< endl << states_est;*/

      //Store center landmark data
      lm_data[it].x = states_est(6);
      lm_data[it].y = states_est(7);

      //Store P landmark data
      P_lm_data[it] = P;

      //Update Rxio data
      Rxio(seq(6,7),all) = R_lm_data[it](seq(6,7),all);
      Rxio(all,seq(6,7)) = R_lm_data[it](all,seq(6,7));

      // estimate radius matrix
      MatrixXf shape_priori;

      shape_priori.resize(int(Aln.rows()),int(Rxio.cols()+ 8));

      shape_priori << Aln*Rxio, 2*Ew_diag;

      Rxio.resize(int(n_states), shape_priori.cols() + n_outputs);

      MatrixXf aux_mat;
      aux_mat.resize(int(n_states), shape_priori.cols() + n_outputs);
      aux_mat << (eye8 - L*C)*shape_priori, -L*2*Ev_diag;

      Rxio = reduction(aux_mat,8);
      R_lm_data[it] = Rxio;

      MatrixXf LimitMatrix = EnvBox(Rxio);

      // Compute the tubes for each state

      int i = 6;

      lm_data[it].max_x = states_est(i) + fabs(LimitMatrix(i,i));
      lm_data[it].min_x = states_est(i) - fabs(LimitMatrix(i,i));

      i = 7;

      lm_data[it].max_y = states_est(i) + fabs(LimitMatrix(i,i));
      lm_data[it].min_y = states_est(i) - fabs(LimitMatrix(i,i));

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
        y_meas.conservativeResize(n_outputs - 2);
        y_meas(0) = sensor.vx;
        y_meas(1) = sensor.psiDot;
        y_meas(2) = sensor.x;
        y_meas(3) = sensor.y;
        y_meas(4) = sensor.yaw;

        // update matrices
        AB_computation(u(0), first_est);

        //L_nlm_computation(u(0));

        // update P matrix

        P_pred(seq(0,5),seq(0,5)) = Aln(seq(0,5),seq(0,5)) * P(seq(0,5),seq(0,5)) * Aln(seq(0,5),seq(0,5)).transpose() + Q(seq(0,5),seq(0,5));

        // compute KF
        MatrixXf x_pred = Aln(seq(0,5),seq(0,5)) *states_est(seq(0,5)) + Bln(seq(0,5),all)*u;

        VectorXf error = y_meas - C(seq(0,4),seq(0,5))*x_pred;

        MatrixXf Z      = C(seq(0,4),seq(0,5))*P_pred(seq(0,5),seq(0,5))*C(seq(0,4),seq(0,5)).transpose() + R(seq(0,4),seq(0,4));
        MatrixXf L_easy = MatrixXf::Zero(6,5);
        L_easy = P_pred(seq(0,5),seq(0,5))*C(seq(0,4),seq(0,5)).transpose()*pseudoInverse(Z);

        P(seq(0,5),seq(0,5)) = P_pred(seq(0,5),seq(0,5)) - L_easy * C(seq(0,4),seq(0,5)) * P_pred(seq(0,5),seq(0,5));

        states_est(seq(0,5)) = x_pred + L_easy*error;

        //resize matrix, might be redundant
        if (Rxio.rows() != 6){

          Rxio.conservativeResize(6,6);

        }

        // Estimate radius matrices
        MatrixXf shape_priori;
        shape_priori.resize(int(Aln.rows()-2),int(Rxio.cols()+ 6));

        shape_priori << Aln(seq(0,5),seq(0,5))*Rxio, Ew_diag(seq(0,5),seq(0,5));

        Rxio.resize(int(n_states-2), shape_priori.cols() + n_outputs-2);

        MatrixXf aux_mat;
        aux_mat.resize(int(n_states)-2, shape_priori.cols() + n_outputs-2);
        aux_mat << (MatrixXf::Identity(6, 6) - L_easy*C(seq(0,4),seq(0,5)))*shape_priori, -L_easy*Ev_diag(seq(0,4),seq(0,4));

        Rxio = reduction(aux_mat,6);

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

        MatrixXf LimitMatrix = EnvBox(Rxio);

        std::vector<float> aux_vec_max;
        std::vector<float> aux_vec_min;

        for (int i = 0; i < 6; i++){

         aux_vec_max.push_back(states_est(i) + fabs(LimitMatrix(i,i)));
         aux_vec_min.push_back(states_est(i) - fabs(LimitMatrix(i,i)));

        }

        //Store interval limits
        upper_limits.push_back(aux_vec_max);
        lower_limits.push_back(aux_vec_min);


      }

    // Initialization function called when created a class
    ObserverLPV_SLAM(){

      //resize matrices
      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
      L_nlm.resize(n_states-2,n_outputs-2);
      Ew.resize(n_states);
      Ev.resize(n_outputs);
      Ew_diag.resize(n_states,n_states);
      Ev_diag.resize(n_outputs,n_outputs);
      u.resize(n_control);
      states_est.resize(n_states);
      Q.resize(n_states,n_states);
      P.resize(n_states,n_states);
      P_pred.resize(n_states,n_states);
      R.resize(n_outputs,n_outputs);

      // load launch information
      n.getParam("lf",lf);
      n.getParam("lr", lr);
      n.getParam("m", m);
      n.getParam("Iz",I);
      n.getParam("Cf",Cf);
      n.getParam("Cr",Cr);
      n.getParam("mu",mu);

      // initialize matrices
      C << 1, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0;

      A = MatrixXf::Zero(n_states,n_states);
      Aln = A;
      P = MatrixXf::Zero(n_states,n_states);


      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      Bln = B;

      Q << 0.01252*0.01252,   0,               0,                0,               0,               0,               0,       0,
           0,                 0.0152*0.0152,   0,                0,               0,               0,               0,       0,
           0,                 0,               0.1252*0.1252,    0,               0,               0,               0,       0,
           0,                 0,               0,                0.0034*0.0034,   0,               0,               0,       0,
           0,                 0,               0,                0,               0.0034*0.0034,   0,               0,       0,
           0,                 0,               0,                0,               0,               0.0034*0.0034,   0,       0,
           0,                 0,               0,                0,               0,               0,               0.0,     0,
           0,                 0,               0,                0,               0,               0,               0,       0.0;


      R << 0.1*0.1,  0,         0,        0,        0,         0,         0,
           0,        0.1*0.1,   0,        0,        0,         0,         0,
           0,        0,         0.2*0.2,  0,        0,         0,         0,
           0,        0,         0,        0.2*0.2,  0,         0,         0,
           0,        0,         0,        0,        0.2*0.2,   0,         0,
           0,        0,         0,        0,        0,         0.01,     0,
           0,        0,         0,        0,        0,         0,         0.01;

      u  << 0,0;
      Ew << 0.01252 , 0.0152, 0.1252, 0.0034 ,0.0034  , 0.0034 , 0, 0;
      Ev << 0.1 , 0.1, 0.2 ,0.2 , 0.2, 0.1, 0.1;
      Ew_diag = MatrixXf::Zero(n_states,n_states);
	    Ev_diag = MatrixXf::Zero(n_outputs,n_outputs);



      for (int i=0; i<n_states; i++){

        Ew_diag(i,i) = Ew(i);

      }

      for (int i=0; i<n_outputs; i++){

        Ev_diag(i,i) = Ev(i);

      }

      states_est << vx, vy, psiDot, x, y, yaw, 0, 0;

      // Load gain matrices, if using the LMI
      //LoadMatrices();
      //LoadMatrices_nlm();

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
    void LoadMatrices(){
      estimator::lmi_data srv;
      srv.request.est_id = "ZKF_SLAM";
      client.call(srv);

      ROS_ERROR_STREAM("Loading" << srv.response.L.size() << " matrices ");
      Llmi.resize(srv.response.L.size());

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi[i].resize(n_states,n_outputs);

        for (int rows = 0; rows < n_states; rows++){

          for (int cols = 0; cols < n_outputs; cols++){

            Llmi[i](rows,cols) = srv.response.L[i].gains[rows*(n_outputs) + cols];

          }
        }
      }
      std::cout << "end Llmi Gain" << std::endl;
      //fill sched_vars

      sched_vars.resize(srv.response.limits.max.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars[i].push_back(srv.response.limits.min[i]);
        sched_vars[i].push_back(srv.response.limits.max[i]);

      }
    }

    // Load LMI matrices from matlab (if we want to use two diferent gains landmark case and no landmark case)
    void LoadMatrices_nlm(){

      estimator::lmi_data srv;
      srv.request.est_id = "ZKF";
      client.call(srv);

      ROS_ERROR_STREAM("Loading" << srv.response.L.size() << " matrices ");
      Llmi_nlm.resize(srv.response.L.size());

      int n_states_nlm = 6;
      int n_outputs_nlm = 5;

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi_nlm[i].resize(n_states_nlm,n_outputs_nlm);

        for (int rows = 0; rows < n_states_nlm; rows++){

          for (int cols = 0; cols < n_outputs_nlm; cols++){

            Llmi_nlm[i](rows,cols) = srv.response.L[i].gains[rows*(n_outputs_nlm) + cols];

          }
        }
      }
      std::cout << "end Gain" << std::endl;

      //fill sched_vars

	  sched_vars_lm.resize(srv.response.limits.max.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars_lm[i].push_back(srv.response.limits.min[i]);
        sched_vars_lm[i].push_back(srv.response.limits.max[i]);

      }

    }


    void AB_computation(float steer, bool first_est){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;
      B(0,1) = 1;


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

        A = MatrixXf::Zero(n_states,n_states);
        B = MatrixXf::Zero(n_states,2);

      }

      C(5,3) = -cos(yaw);
      C(5,4) = -sin(yaw);
      C(5,6) = cos(yaw);
      C(5,7) = sin(yaw);

      C(6,3) = sin(yaw);
      C(6,4) = -cos(yaw);
      C(6,6) = -sin(yaw);
      C(6,7) = cos(yaw);



      Aln = eye8 + (A*dt);
      Bln = B*dt;

    }


    // LPV interpolation of the gains with landmark
    void L_computation(float steer, float theta_lm){

		float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
		float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
		float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
		float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);
		float M_Stheta_lm       = (sched_vars[7][1] - sin(theta_lm)) / (sched_vars[6][1] - sched_vars[6][0]);
		float M_Ctheta_lm       = (sched_vars[6][1] - cos(theta_lm)) / (sched_vars[7][1] - sched_vars[7][0]);

		VectorXf mu_sch;
		mu_sch.resize(64);

		mu_sch[0]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[1]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[2]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[3]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[4]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[5]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[6]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[7]   = M_vx_despl_min * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[8]   = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[9]   = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[10]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[11]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[12]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[13]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[14]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[15]  = M_vx_despl_min * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

		mu_sch[16]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[17]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[18]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[19]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[20]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[21]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[22]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[23]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[24]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[25]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[26]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[27]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[28]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[29]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[30]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[31]  = M_vx_despl_min * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

		mu_sch[32]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[33]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[34]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[35]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[36]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[37]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[38]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[39]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  M_Stheta_lm                * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[40]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[41]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[42]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[43]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[44]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[45]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[46]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[47]  = (1 - M_vx_despl_min) * M_Ctheta_lm  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

		mu_sch[48]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[49]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[50]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[51]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[52]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[53]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[54]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[55]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  M_Stheta_lm               * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[56]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  M_theta_min;
		mu_sch[57]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
		mu_sch[58]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
		mu_sch[59]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
		mu_sch[60]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
		mu_sch[61]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
		mu_sch[62]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
		mu_sch[63]  = (1 - M_vx_despl_min) * (1 - M_Ctheta_lm)  *  (1 - M_Stheta_lm)         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

		MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

		for (int i = 0; i < 64; i++){

		  result += mu_sch[i] * Llmi[i];

		}

		L = result;
    }

    // LPV interpolation of the gains without landmark

    void L_nlm_computation(float steer){


      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw)   / (sched_vars[5][1] - sched_vars[5][0]);

      VectorXf mu_sch;
      mu_sch.resize(16);

      mu_sch[0]               = M_vx_despl_min         * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[1]               = M_vx_despl_min         * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[2]               = M_vx_despl_min         * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[3]               = M_vx_despl_min         * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[4]               = M_vx_despl_min         * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[5]               = M_vx_despl_min         * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[6]               = M_vx_despl_min         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[7]               = M_vx_despl_min         * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

      mu_sch[8]               = (1-M_vx_despl_min)     * M_vy_despl_min      * M_steer_min      *  M_theta_min;
      mu_sch[9]               = (1-M_vx_despl_min)     * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min);
      mu_sch[10]              = (1-M_vx_despl_min)     * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min;
      mu_sch[11]              = (1-M_vx_despl_min)     * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
      mu_sch[12]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min;
      mu_sch[13]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
      mu_sch[14]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min;
      mu_sch[15]              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);

      MatrixXf result = MatrixXf::Zero(n_states-4,n_outputs-2);

      for (int i = 0; i < 16; i++){

          result += mu_sch[i] * Llmi_nlm[i];

      }

      L_nlm = -result*dt;
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

    error_pub = n.advertise<estimator::ErrorsInfo>("errorsEKF", 1);
    data_pub = n.advertise<estimator::DataInfo>("dataEKF", 1);

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

  ros::init(argc, argv, "ekf");
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

  while (ros::ok() && i_it < 4000)
  {
    flag = 1;
    if (flag && (actuators.a != 0) && (actuators.steer != 0) && (truth.vx > 1)){

        if (i_it == 0){

          // initialize estimator matrices
          start = std::chrono::steady_clock::now();

          markers.GetMarkersStart();

          observer.states_est(3)      = truth.x;
          observer.states_est(4)      = truth.y;
          observer.states_est(5)      = truth.yaw;
          observer.states_est(0)      = truth.vx;
          observer.states_est(1)      = truth.vy;
          observer.states_est(2)      = truth.psiDot;

        }

        x_t   = truth.x;
		y_t   = truth.y;
        yaw_t = truth.yaw;
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


	    /*ROS_ERROR_STREAM("update finished: x " << observer.x <<" y "<< observer.y <<" yaw " << observer.yaw <<" vx " << observer.vx
	                <<" vy "<< observer.vy <<" psiDot " << observer.psiDot );


        ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                          <<" vy "<< truth.vy <<" psiDot " << truth.psiDot << "iteration " << i_it);*/
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
