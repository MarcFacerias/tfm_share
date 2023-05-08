#define _GLIBCXX_USE_C99 1
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <estimator/DataInfo.h>
#include <estimator/ErrorsInfo.h>
#include <estimator/SendCov.h>
#include <Eigen/Dense>
#include <ctime>
#include <math.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <estimator/lmi_data.h>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/QR>
using namespace Eigen;

/*

Implementation of a classic EKF 

*/

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

///////////////////////////////////////////////////////////////////////////////
                                // GLOBAL //
///////////////////////////////////////////////////////////////////////////////ObserverLPV

float dt;
bool flag = 0;

///////////////////////////////////////////////////////////////////////////////
                             // CALLBACK and AUX //
///////////////////////////////////////////////////////////////////////////////
class GetSensorData{

  public:
    float x = 0;
    float y = 0;
    float yaw = 0;
    float psiDot = 0;
    float vx = 0;
    float vy = 0;
    ros::NodeHandle n;
    ros::Subscriber pose_sub;

    GetSensorData(){

      pose_sub = n.subscribe("/sensorStates", 1000, &GetSensorData::GetPoseCallback, this);

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

      groundTruth_sub = n.subscribe("/vehicle_state", 1000, &GetGroundTruthData::GetGroundTruthDataCallback, this);

    }

    void GetGroundTruthDataCallback(const l4vehicle_msgs::VehicleState& msg)
    {

        x       = msg.x;
        y       = msg.y;
        yaw     = msg.heading;
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

      actu_sub = n.subscribe("/ecu", 1000, &GetActuData::GetActuCallback, this);

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

MatrixXf EnvBox(MatrixXf Rxxio){

  int rows = Rxxio.rows();
  std::cout << rows << " " << Rxxio.cols() << std::endl;
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

class ObserverLPV{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    estimator::lmi_data srv;
    ros::Publisher cov_pub;

    // model dimension variables
    float n_states = 6;
    float n_outputs = 5;
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

    float x      = 0.02;
    float y      = 0.0;
    float vx     = 0.75;
    float vy     = 0.0;
    float yaw    = 0.0;
    float psiDot = 0.0;

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf L;
    MatrixXf P;
    MatrixXf P_pred;
    MatrixXf Q;
    MatrixXf R;

    //Vectors
    VectorXf Ew;
    VectorXf Ev;
    VectorXf u;

    //estimated states
    VectorXf states_est;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;
    std::list <std::vector<float>> est_error_hist;

    void estimateState(GetSensorData sensor, GetActuData ecu){

      VectorXf y_meas;
      y_meas.resize(5);

      y_meas(0) = sensor.vx;
      y_meas(1) = sensor.psiDot;
      y_meas(2) = sensor.x;
      y_meas(3) = sensor.y;
      y_meas(4) = sensor.yaw;

      u(0)      = ecu.steer;
      u(1)      = ecu.a;

      // update matrices
      AB_computation(u(0));

      //MatrixXf aux_tf = Aln.transpose();
      P_pred = Aln * P * Aln.transpose() + Q;

      MatrixXf x_pred = states_est + nl_sys()*dt;

      VectorXf error = y_meas - C*x_pred;

      MatrixXf Z     = C*P_pred*C.transpose() + R;
      L              = P_pred*C.transpose()*pseudoInverse(Z);
      P              = P_pred - L * C * P_pred;  

      states_est = x_pred + L*error;


      vx      = states_est(0);
      vy      = states_est(1);
      psiDot  = states_est(2);
      x       = states_est(3);
      y       = states_est(4);
      yaw     = states_est(5);

      ROS_ERROR_STREAM("update finished: x " << x <<" y "<< y <<" yaw " << yaw <<" vx " << vx
                          <<" vy "<< vy <<" yaw " <<" psiDot " << psiDot);

      x_est_hist.push_back(x);
      y_est_hist.push_back(y);
      yaw_est_hist.push_back(yaw);
      vx_est_hist.push_back(vx);
      vy_est_hist.push_back(vy);
      psiDot_est_hist.push_back(psiDot);

      estimator::SendCov msg;

      for (int i = 0; i< 6; i++){

        for (int j = 0; j< 6; j++){

          msg.matrix.push_back(P(i,j));

        }
      }

      msg.folder = "EKF";
      cov_pub.publish(msg);

    }

    ObserverLPV(){

      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
      Q.resize(n_states,n_states);
      P.resize(n_states,n_states);
      R.resize(n_outputs,n_outputs);
      P_pred.resize(n_states,n_states);
      R.resize(n_outputs,n_outputs);
      u.resize(n_control);
      states_est.resize(n_states);

      n.getParam("lf",lf);
      n.getParam("lr", lr);
      n.getParam("m", m);
      n.getParam("Iz",I);
      n.getParam("Cf",Cf);
      n.getParam("Cr",Cr);
      n.getParam("mu",mu);

      C << 1, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0,
           0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 1;

      Q << 0.0001, 0, 0, 0, 0, 0,
           0, 0.00010, 0, 0, 0, 0,
           0, 0, 0.007, 0, 0, 0,
           0, 0, 0, 0.0001, 0, 0,
           0, 0, 0, 0, 0.00001, 0,
           0, 0, 0, 0, 0, 0.00001;

      R << 0.0028, 0, 0, 0, 0,
           0, 0.0005, 0, 0, 0,
           0, 0, 0.0005, 0, 0,
           0, 0, 0, 0.0005, 0,
           0, 0, 0, 0, 0.0034;

      A << 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0;

      P = A;

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      u << 0,0;

      states_est << vx, vy, psiDot, x, y, yaw;
      cov_pub = n.advertise<estimator::SendCov>("Cov", 5);
    }

    private:

    VectorXf nl_sys(){

      VectorXf dx = VectorXf::Zero(6);
      
      float    a           = u(1);
      float    delta       = u(0);

      float FyF = -Cf * atan((vy + lf*psiDot)/vx) - delta;
      float FyR = -Cr * atan((vy - lr*psiDot)/vx);


      dx(0) = a -mu*vx - FyF*sin(delta)/m  +  psiDot*vy ;    
      dx(1) = ( FyF*cos(delta) + FyR ) / m  -  psiDot*vx ; 
      dx(2) = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
      dx(3) = vx*cos(yaw) - vy*sin(yaw);
      dx(4) = vx*sin(yaw) + vy*cos(yaw);
      dx(5) = psiDot;

      return dx;

    }

    // void AB_computation(MatrixXf &A, MatrixXf &B, float theta, float steer, float vx, float vy){
    void AB_computation(float theta){

      //update B
      B(0,0) = Cf*(cos(theta)*vy - sin(theta)*vx -cos(theta)*theta*vx + cos(theta)*lf*psiDot)/(m*vx);
      B(0,1) = 1;
      B(1,0) = Cf*(cos(theta)*vx + sin(theta)*vy -sin(theta)*theta*vx + sin(theta)*lf*psiDot)/(m*vx);
      B(2,0) = lf*Cf*(cos(theta)*vx - sin(theta)*vy -sin(theta)*theta*vx + sin(theta)*lf*psiDot)/(I*vx);
      // B(1,1) = cos(steer);
      // B(2,1) = sin(steer);

      //Update A
      A(0,0) =  -mu -(sin(theta) * Cf *(vx+lf*psiDot))/(m*vx*vx);
      A(0,1) =  psiDot + (sin(theta) * Cf)/(m*vx);
      A(0,2) =  vy + (sin(theta) * lf * Cf)/(m*vx);

      A(1,0) =  vy*(Cr + cos(theta)*Cf)/(m*vx*vx) - psiDot*((Cr*lr - cos(theta)*Cf*lf)/(m*vx*vx)+1);
      A(1,1) =  - (Cr + cos(theta)*Cf)/(m*vx);
      A(1,2) =  (Cr*lr - cos(theta)*Cf*lf)/(m*vx) - vx;

      A(2,0) =  psiDot*(cos(theta)*Cf*lf*lf +Cr*lr*lr)/(I*vx*vx) - vy*(Cr*lr - cos(theta)*Cf*lf)/(I*vx*vx);
      A(2,1) =  (Cr*lr - cos(theta)*Cf*lf)/(I*vx);
      A(2,2) =  -(Cr*lr*lr - cos(theta)*Cf*lf*lf)/(I*vx);

      A(3,0) =  cos(yaw);
      A(3,1) =  -sin(yaw);
      A(3,5) =  -cos(yaw)*vy -sin(yaw)*vx;

      A(4,0) =  sin(yaw);
      A(4,1) =  cos(yaw);
      A(4,5) =  cos(yaw)*vy -sin(yaw)*vx;


      Aln = eye6 + (A*dt);
      Bln = B*dt;

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
  void compute_error(GetGroundTruthData truth,ObserverLPV estimator){

    estimator::ErrorStates temp;

    temp.vx = truth.vx - estimator.vx;
    temp.vy = truth.vy - estimator.vy;
    temp.psiDot = truth.psiDot - estimator.psiDot;
    temp.x = truth.x - estimator.x;
    temp.y = truth.y - estimator.y;
    temp.psi = truth.yaw - estimator.yaw;

    ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                      <<" vy "<< truth.vy <<" yaw " << truth.yaw <<" psiDot " << truth.psiDot);

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

  void save_error(GetGroundTruthData truth,ObserverLPV estimator, std::string folder){

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

    msg.folder = folder;
    msg_data.folder = folder;

    error_pub.publish(msg);
    data_pub.publish(msg_data);

  }

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "class_ekf");
  ros::NodeHandle n;

  ros::Subscriber actu_flag = n.subscribe("flag", 1000, FlagCallback);
  //ros::Publisher est_pub = n.advertise<lpv_mpc::pos_info>("/pos_info", 1); //Close Estimation Loop

  n.getParam("dt", dt);
  dt = 0.005;
  ros::Rate loop_rate(200);

  ros::service::waitForService("LoadMatrices", -1);

  ObserverLPV observer; //EKF_LPV
  //ObserverLPVi observer; //EKF_LPVi
  //ObserverLPV_UIO observer;

  GetActuData actuators;
  GetSensorData sensors;
  ComputeError error_manager;
  GetGroundTruthData truth;

  int i_it = 0;

  std::cout << "EKF" << std::endl;
  while (ros::ok() && i_it < 2500)
  {

    if (flag && (actuators.a != 0) && (actuators.steer != 0) && (truth.vx > 1)){

        if (i_it == 0){

          observer.states_est(3)      = truth.x;
          observer.states_est(4)      = truth.y;
          observer.states_est(5)      = truth.yaw;
          observer.states_est(0)      = truth.vx;
          observer.states_est(1)      = truth.vy;
          observer.states_est(2)      = truth.psiDot;

        }

        auto start = std::chrono::steady_clock::now();

        observer.estimateState(sensors,actuators);

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

        auto end = std::chrono::steady_clock::now();

        std::cout << "Elapsed time in milliseconds : "
                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
                  << " µs" << std::endl;
        //error_manager.compute_error(truth, observer);
        i_it++;
        // std::cout << i_it << std::endl;
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

  error_manager.save_error(truth, observer, "EKF");


  return 0;
}
