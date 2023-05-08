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
float dt;
bool flag = 0;

/*


Library containing the implementation of each estimator (label along the name of the class) and some
funtions used to deal with zonotopes


*/
/////////////////////////struct//////////////////////////

//structure of a sensor measuremnt
struct PolarPose{

  float range;
  float angle;
  uint id;

};

//Pose of a landmark
struct GlobalPose{

  float x;
  float y;
  uint id;

};

/////////////////////////functions//////////////////////////

// Pseudoinverse
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

//Cartesian to polar coordinates
void Polar(float x, float y, PolarPose *pose){

  pose -> range     = sqrt(x*x + y*y);
  pose -> angle     = atan(abs(y) / abs(x));

}

// bounding box of a zonotope
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

// Index sorting as proposed by Combastel

VectorXf sort_indexes(const VectorXf v) {

  // initialize original index locations
  VectorXf idx = VectorXf::LinSpaced(v.size()+1, 0, v.size());

  std::stable_sort(idx.data(), idx.data() + idx.size()-1,
       [v](int i1, int i2) {return v(i1) < v(i2);});

  return idx;
}

//Matrix complexity reduction

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

    Out.resize(6,idx_s2.size());

    for (int i = 0; i < idx_s2.size(); i++){

      Out.col(i) = Rxio.col(idx_s2(i));

    }

  }

  if (idx_s1.size() != 0){

    MatrixXf Ms;
    Ms.resize(6,idx_s1.size());

    for (int i = 0; i < idx_s1.size(); i++){

      Ms.col(i)  = Rxio.col(idx_s1(i));


    }

    Ms = EnvBox(Ms);

    if (Out.cols() > 0){

      Out.conservativeResize(6,Out.cols() + idx_s1.size());

      Out(all, seq(Out.cols() - idx_s2.size(),Out.cols() )) = Ms;

    } 
    else {

      Out.conservativeResize(6, idx_s1.size());
      Out = Ms;

    } 

  }


  return Out;

}

// start flag subscriber
void FlagCallback(const std_msgs::Bool& msg){

  flag = msg.data;

}

////////////////////////////classes////////////////////////////

//markers subscriber
class GetMarkers{

  private:

    ros::NodeHandle n;
    ros::Subscriber marker_sub;

  public:

    std::deque<PolarPose> landmark_list;

    GetMarkers(){

      marker_sub = n.subscribe("/Corrected_Pose", 1, &GetMarkers::GetMarkersCallback, this);

    }

    void GetMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
    {   

      for (int i = 0; i < msg.markers.size(); i++){

        PolarPose pose;

        Polar(msg.markers[i].pose.pose.position.x, msg.markers[i].pose.pose.position.y, &pose);
        pose.id = msg.markers[i].id;
        landmark_list.push_back(pose);


      }
    }

};

//sensor subscriber
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

//ground truth subscriber
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

//Actuator subscriber
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

//LPV LMI Observer
class ObserverLPV{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    estimator::lmi_data srv;

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

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf L;
    MatrixXf Rxxio;
    MatrixXf shape_priori;

    //Vectors
    VectorXf u;
    VectorXf mu_sch;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<std::vector<float>> sched_vars;

    //estimated states
    float x      = 0.02;
    float y      = 0.0;
    float vx     = 0.75;
    float vy     = 0.0;
    float yaw    = 0.0;
    float psiDot = 0.0;
    VectorXf states_est;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;

    void estimateState(GetSensorData sensor, GetActuData ecu){

      VectorXf y_meas;

      y_meas.resize(n_outputs);
      y_meas(0) = sensor.vx;
      y_meas(1) = sensor.psiDot;
      y_meas(2) = sensor.x;
      y_meas(3) = sensor.y;
      y_meas(4) = sensor.yaw;

      u(0)      = ecu.steer;
      u(1)      = ecu.a;

      // update matrices
      AB_computation(u(0));

      L_computation(u(0));
      //priori estimation

      MatrixXf prior = Aln*states_est + Bln*u;

      states_est = prior + L * (y_meas - C*prior);

      //save states
      vx      = states_est(0);
      vy      = states_est(1);
      psiDot  = states_est(2);
      x       = states_est(3);
      y       = states_est(4);
      yaw     = states_est(5);

      ROS_ERROR_STREAM("update finished: x " << x <<" y "<< y <<" yaw " << yaw <<" vx " << vx
                        <<" vy "<< vy <<" yaw " << yaw <<" psiDot " << psiDot);

      x_est_hist.push_back(x);
      y_est_hist.push_back(y);
      yaw_est_hist.push_back(yaw);
      vx_est_hist.push_back(vx);
      vy_est_hist.push_back(vy);
      psiDot_est_hist.push_back(psiDot);

    }

    ObserverLPV(){

      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
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

      A << 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0;

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      u << 0,0;

      states_est << vx, vy, psiDot, x, y, yaw;
      LoadMatrices();
    }

    private:

    void LoadMatrices(){

      srv.request.est_id = "LPV";
      client.call(srv);

      ROS_ERROR_STREAM("resize v" << srv.response.L.size());
      Llmi.resize(srv.response.L.size());

      // fill LMIs
      for (int i = 0; i < srv.response.L.size(); i++){

        Llmi[i].resize(n_states,n_outputs);

        for (int rows = 0; rows < n_states; rows++){

          for (int cols = 0; cols < n_outputs; cols++){

            Llmi[i](rows,cols) = srv.response.L[i].gains[rows*(n_outputs) + cols];

          }
        }

        //std::cout << Llmi[i] << std::endl;

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
    void AB_computation(float steer){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;
      B(1,1) = cos(steer);
      B(2,1) = sin(steer);

      //Update A
      A(0,0) =  -mu;
      A(0,1) = (sin(steer) * Cf) / (m*vx);
      A(0,2) = ((sin(steer) * Cf * lf) / (m*vx)) + vy;
      A(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
      A(1,2) = -((lf * Cf * cos(steer) - lr * Cr) / (m*vx)) - vx;
      A(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
      A(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);
      A(3,0) = cos(yaw);
      A(4,0) = sin(yaw);
      A(3,1) = -sin(yaw);
      A(4,1) = cos(yaw);
      A(5,2) = 1;

      Aln = eye6 + (A*dt);
      Bln = B*dt;
    }

    // void L_computation(MatrixXf &L, float vx, float vy, float theta, float steer){
    void L_computation(float steer){

      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);

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

      MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

      for (int i = 0; i < 16; i++){

          result += mu_sch[i] * Llmi[i];

      }

      L = -result*dt;
    }
};

//LPV LMI Iterative Observer
class ObserverLPV_it{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    estimator::lmi_data srv;

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

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf L;
    MatrixXf Rxxio;
    MatrixXf shape_priori;
    MatrixXf P;
    MatrixXf R;
    MatrixXf Q;
    MatrixXf P_pred;

    //Vectors
    VectorXf u;
    VectorXf mu_sch;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<std::vector<float>> sched_vars;

    //estimated states
    float x      = 0.02;
    float y      = 0.0;
    float vx     = 0.75;
    float vy     = 0.0;
    float yaw    = 0.0;
    float psiDot = 0.0;
    VectorXf states_est;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;

    void estimateState(GetSensorData sensor, GetActuData ecu){

      VectorXf y_meas;

      y_meas.resize(n_outputs);
      y_meas(0) = sensor.vx;
      y_meas(1) = sensor.psiDot;
      y_meas(2) = sensor.x;
      y_meas(3) = sensor.y;
      y_meas(4) = sensor.yaw;

      u(0)      = ecu.steer;
      u(1)      = ecu.a;

      // update matrices
      AB_computation(u(0));

      //L_computation(u(0));
      //priori estimation


      P_pred = Aln * P * Aln.transpose() + Q;

      MatrixXf x_pred = Aln*states_est + Bln*u;

      VectorXf error = y_meas - C*x_pred;

      MatrixXf Z     = C*P_pred*C.transpose() + R;
      L              = P_pred*C.transpose()*pseudoInverse(Z);
      P              = P_pred - L * C * P_pred;  

      states_est = x_pred + L*error;

      //save states
      vx      = states_est(0);
      vy      = states_est(1);
      psiDot  = states_est(2);
      x       = states_est(3);
      y       = states_est(4);
      yaw     = states_est(5);

      /*ROS_ERROR_STREAM("update finished: x " << x <<" y "<< y <<" yaw " << yaw <<" vx " << vx
                        <<" vy "<< vy <<" yaw " << yaw <<" psiDot " << psiDot);*/
      x_est_hist.push_back(x);
      y_est_hist.push_back(y);
      yaw_est_hist.push_back(yaw);
      vx_est_hist.push_back(vx);
      vy_est_hist.push_back(vy);
      psiDot_est_hist.push_back(psiDot);

    }

    ObserverLPV_it(){

      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
      u.resize(n_control);
      states_est.resize(n_states);
      Q.resize(n_states,n_states);
      P.resize(n_states,n_states);
      R.resize(n_outputs,n_outputs);
      P_pred.resize(n_states,n_states);

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

      A << 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0;

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      u << 0,0;

      Q << 0.000025, 0, 0, 0, 0, 0,
           0, 0.000020, 0, 0, 0, 0,
           0, 0, 0.0069, 0, 0, 0,
           0, 0, 0, 0.0000125, 0, 0,
           0, 0, 0, 0, 0.00001156, 0,
           0, 0, 0, 0, 0, 0.000025;

      R << 0.0053*0.0053, 0, 0, 0, 0,
           0, 0.0064, 0, 0, 0,
           0, 0, 0.0012, 0, 0,
           0, 0, 0, 0.0012, 0,
           0, 0, 0, 0, 0.0076;

      P = A;

      states_est << vx, vy, psiDot, x, y, yaw;
      LoadMatrices();
    }

    private:

    void LoadMatrices(){

      srv.request.est_id = "LPV";
      //srv.request.est_id = "ZKF";
      client.call(srv);

      ROS_ERROR_STREAM("resize v" << srv.response.L.size());
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

      //fill sched_vars

      sched_vars.resize(srv.response.limits.max.size());

      mu_sch.resize(srv.response.L.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars[i].push_back(srv.response.limits.min[i]);
        sched_vars[i].push_back(srv.response.limits.max[i]);

      }

    }

    // void AB_computation(MatrixXf &A, MatrixXf &B, float theta, float steer, float vx, float vy){
    void AB_computation(float steer){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;
      B(1,1) = cos(steer);
      B(2,1) = sin(steer);

      //Update A
      A(0,0) =  0;
      A(0,1) = (sin(steer) * Cf) / (m*vx);
      A(0,2) = ((sin(steer) * Cf * lf) / (m*vx)) + vy;
      A(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
      A(1,2) = -((lf * Cf * cos(steer) - lr * Cr) / (m*vx)) - vx;
      A(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
      A(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);
      A(3,0) = cos(yaw);
      A(4,0) = sin(yaw);
      A(3,1) = -sin(yaw);
      A(4,1) = cos(yaw);
      A(5,2) = 1;

      Aln = eye6 + (A*dt);
      Bln = B*dt;
    }

    // void L_computation(MatrixXf &L, float vx, float vy, float theta, float steer){
    void L_computation(float steer){

      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);

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

      MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

      for (int i = 0; i < 16; i++){

          result += mu_sch[i] * Llmi[i];

      }

      L = -result*dt;
    }
};

//LPV LMI ZKF Observer
class ObserverLPVi{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    estimator::lmi_data srv;

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

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf Ew_diag = MatrixXf::Zero(n_states, n_states);
    MatrixXf Ev_diag = MatrixXf::Zero(n_outputs,n_outputs);
    MatrixXf L;
    MatrixXf Rxio = MatrixXf::Zero(n_states, n_states);

    //Vectors
    VectorXf Ew;
    VectorXf Ev;
    VectorXf u;
    VectorXf mu_sch;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<std::vector<float>> sched_vars;

    //estimated states
    float x      = 0.0;
    float y      = 0.0;
    float vx     = 1.0112;
    float vy     = 0.0115;
    float yaw    = 0.0;
    float psiDot = 0.0946;
    VectorXf states_est;


    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;
    std::list <std::vector<float>> upper_limits;
    std::list <std::vector<float>> lower_limits;

    void estimateState(GetSensorData sensor, GetActuData ecu){

      VectorXf y_meas;

      y_meas.resize(n_outputs);
      y_meas(0) = sensor.vx;
      y_meas(1) = sensor.psiDot;
      y_meas(2) = sensor.x;
      y_meas(3) = sensor.y;
      y_meas(4) = sensor.yaw;

      u(0)      = ecu.steer;
      u(1)      = ecu.a;

      // update matrices
      AB_computation(u(0));

      L_computation(u(0));
      //priori estimation


      //Center
      MatrixXf prior = Aln*states_est + Bln*u;

      states_est = prior + L * (y_meas - C*prior);

      //Expansion Matrices
      MatrixXf shape_priori;
      shape_priori.resize(int(Aln.rows()),int(Rxio.cols()+ n_states));

      shape_priori << Aln*Rxio, 2*Ew_diag;

      Rxio.resize(int(n_states), shape_priori.cols() + n_outputs );

      MatrixXf aux_mat;
      aux_mat.resize(int(n_states), shape_priori.cols() + n_outputs);
      aux_mat << (eye6 - L*C)*shape_priori, -L*2*Ev_diag;
      Rxio = reduction(aux_mat,6);


      MatrixXf LimitMatrix = EnvBox(Rxio);

      std::vector<float> aux_vec_max;
      std::vector<float> aux_vec_min;

      for (int i = 0; i < n_states; i++){

        aux_vec_max.push_back(states_est(i) + fabs(LimitMatrix(i,i)));
        aux_vec_min.push_back(states_est(i) - fabs(LimitMatrix(i,i)));

      }

      upper_limits.push_back(aux_vec_max);
      lower_limits.push_back(aux_vec_min);

      //save states
      vx      = states_est(0);
      vy      = states_est(1);
      psiDot  = states_est(2);
      x       = states_est(3);
      y       = states_est(4);
      yaw     = states_est(5);

        ROS_ERROR_STREAM("update finished: x " << x <<" y "<< y <<" yaw " << yaw <<" vx " << vx
                          <<" vy "<< vy <<" yaw " <<" psiDot " << psiDot);
      
        // ROS_ERROR_STREAM("upper limits: x " << aux_vec_max[3] <<" y "<< aux_vec_max[4] <<" yaw " << aux_vec_max[5] <<" vx " << aux_vec_max[0]
        //                   <<" vy "<< aux_vec_max[1] <<" PsiYaw " <<" psiDot " << aux_vec_max[2]);
        // ROS_ERROR_STREAM("lower limits: x " << aux_vec_min[3] <<" y "<< aux_vec_min[4] <<" yaw " << aux_vec_min[5] <<" vx " << aux_vec_min[0]
        //                   <<" vy "<< aux_vec_min[1] <<" PsiYaw " <<" psiDot " << aux_vec_min[2]);

        /*std::cout << "Lower bounds " << aux_vec_min[0] << " " << aux_vec_min[1] << " " << aux_vec_min[2] << " "
                  << aux_vec_min[3] << " " << aux_vec_min[4] << " " << aux_vec_min[5] << " " << std::endl;

        std::cout << "Upper bounds " << aux_vec_max[0] << " " << aux_vec_max[1] << " " << aux_vec_max[2] << " "
                  << aux_vec_max[3] << " " << aux_vec_max[4] << " " << aux_vec_max[5] << " " << std::endl;*/

      x_est_hist.push_back(x);
      y_est_hist.push_back(y);
      yaw_est_hist.push_back(yaw);
      vx_est_hist.push_back(vx);
      vy_est_hist.push_back(vy);
      psiDot_est_hist.push_back(psiDot);

    }

    ObserverLPVi(){

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

      n.getParam("lf", lf);
      n.getParam("lr", lr);
      n.getParam("m", m);
      n.getParam("Iz", I);
      n.getParam("Cf", Cf);
      n.getParam("Cr", Cr);
      n.getParam("mu", mu);

      C << 1, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0,
           0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 1;

      A << 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0;

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      u << 0,0;

      Ew << 0.01252,  0.0152,  0.1252, 0.004, 0.004, 0.008 ;
      Ev << 0.08 ,0.035,  0.035,  0.035,  0.087;

      states_est << vx, vy, psiDot, x, y, yaw;

      for (int i=0; i<n_states; i++){

        Ew_diag(i,i) = Ew(i);

      }

      for (int i=0; i<n_outputs; i++){

        Ev_diag(i,i) = Ev(i);

      }

      LoadMatrices();
    }

    private:

    void LoadMatrices(){

      srv.request.est_id = "ZKF";
      client.call(srv);

      ROS_ERROR_STREAM("resize v" << srv.response.L.size());
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

      //fill sched_vars

      sched_vars.resize(srv.response.limits.max.size());

      mu_sch.resize(srv.response.L.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars[i].push_back(srv.response.limits.min[i]);
        sched_vars[i].push_back(srv.response.limits.max[i]);

      }

    }

    // void AB_computation(MatrixXf &A, MatrixXf &B, float theta, float steer, float vx, float vy){
    void AB_computation(float steer){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;
      B(1,1) = cos(steer);
      B(2,1) = sin(steer);

      //Update A
      A(0,0) =  0;
      A(0,1) = (sin(steer) * Cf) / (m*vx);
      A(0,2) = ((sin(steer) * Cf * lf) / (m*vx)) + vy;
      A(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
      A(1,2) = -((lf * Cf * cos(steer) - lr * Cr) / (m*vx)) - vx;
      A(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
      A(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);
      A(3,0) = cos(yaw);
      A(4,0) = sin(yaw);
      A(3,1) = -sin(yaw);
      A(4,1) = cos(yaw);
      A(5,2) = 1;

      Aln = eye6 + (A*dt);
      Bln = B*dt;
    }

    // void L_computation(MatrixXf &L, float vx, float vy, float theta, float steer){
    void L_computation(float steer){

      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);

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

      MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

      for (int i = 0; i < 16; i++){

          result += mu_sch[i] * Llmi[i];

      }

      L = -result*dt;
    }
};

//LPV LMI EKF UIO Observer
class ObserverLPV_UIO{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    estimator::lmi_data srv;
    

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

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf A_uio;
    MatrixXf B_uio;
    MatrixXf L;
    MatrixXf E_diag = MatrixXf::Zero(n_states,n_states);

    //Vectors
    VectorXf Ew;
    VectorXf Ev;
    VectorXf E;
    VectorXf u;
    VectorXf y_ant = VectorXf::Zero(n_outputs);
    MatrixXf tau;
    VectorXf mu_sch;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<std::vector<float>> sched_vars;

    //estimated states
    float x      = 0.0;
    float y      = 0.0;
    float vx     = 1.0112;
    float vy     = 0.0115;
    float yaw    = 0.0;
    float psiDot = 0.0946;
    VectorXf states_est;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;
    std::list <MatrixXf> dis_est_max_hist;
    std::list <MatrixXf> dis_est_min_hist;

    void estimateState(GetSensorData sensor, GetActuData ecu){

      VectorXf y_meas;

      y_meas.resize(n_outputs);
      y_meas(0) = sensor.vx;
      y_meas(1) = sensor.psiDot;
      y_meas(2) = sensor.x;
      y_meas(3) = sensor.y;
      y_meas(4) = sensor.yaw;

      u(0)      = ecu.steer;
      u(1)      = ecu.a;

      // update matrices
      AB_computation(u(0));

      L_computation(u(0));

      //dist estimation
      MatrixXf dis_est_max = tau*(y_meas - C*(Aln*states_est + Bln*u + Ew) - Ev);
      MatrixXf dis_est_min = tau*(y_meas - C*(Aln*states_est + Bln*u - Ew) + Ev);

      //priori estimation    
      MatrixXf prior = (eye6 - E_diag*tau*C)*states_est + A_uio*states_est + B_uio*u + E_diag*tau*y_meas ;

      states_est = prior + L * (y_meas - C*prior);

      //save states
      vx      = states_est(0);
      vy      = states_est(1);
      psiDot  = states_est(2);
      x       = states_est(3);
      y       = states_est(4);
      yaw     = states_est(5);

      y_ant = y_meas;

      //std::cout << dis_est(0,0) << std::endl;
      ROS_ERROR_STREAM("update finished: x " << x <<" y "<< y <<" yaw " << yaw <<" vx " << vx
              <<" vy "<< vy <<" yaw " << yaw <<" psiDot " << psiDot);

      x_est_hist.push_back(x);
      y_est_hist.push_back(y);
      yaw_est_hist.push_back(yaw);
      vx_est_hist.push_back(vx);
      vy_est_hist.push_back(vy);
      psiDot_est_hist.push_back(psiDot);
      dis_est_max_hist.push_back(dis_est_max);
      dis_est_min_hist.push_back(dis_est_min);
      
    }

    ObserverLPV_UIO(){

      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      A_uio.resize(n_states,n_states);
      B_uio.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
      Ew.resize(n_states);
      E.resize(n_states);
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

      y_ant(0)      = vx;
      y_ant(1)      = psiDot;
      y_ant(2)      = x;
      y_ant(3)      = y;
      y_ant(4)      = yaw;


      C << 1, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0,
           0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 1;

      A << 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0;

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      u << 0,0;

      Ew << 0.01252/2,  0.0152, 0.1252, 0.004, 0.004, 0.008 ;
      Ev << 0.008   ,0.08   ,0.08  ,0.08  ,0.087;
      E  << -1/m  ,0.0   ,0.0  ,0.0  ,0.0 ,0.0 ;
 

      for (int i=0; i<n_states; i++){

        E_diag(i,i) = E(i);

      }

      MatrixXf aux_pinv = C*E_diag;
      tau = pseudoInverse(aux_pinv);


      states_est << vx, vy, psiDot, x, y, yaw;
      LoadMatrices();
    }

    private:

      int iterations = 0;

      void LoadMatrices(){

        srv.request.est_id = "LPV_UIO";
        client.call(srv);

        ROS_ERROR_STREAM("resize v" << srv.response.L.size());
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

      //fill sched_vars

      sched_vars.resize(srv.response.limits.max.size());

      mu_sch.resize(srv.response.L.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars[i].push_back(srv.response.limits.min[i]);
        sched_vars[i].push_back(srv.response.limits.max[i]);

      }

    }

    void AB_computation(float steer){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;
      B(1,1) = cos(steer);
      B(2,1) = sin(steer);

      //Update A
      A(0,0) =  0;
      A(0,1) = (sin(steer) * Cf) / (m*vx);
      A(0,2) = ((sin(steer) * Cf * lf) / (m*vx)) + vy;
      A(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
      A(1,2) = -((lf * Cf * cos(steer) - lr * Cr) / (m*vx)) - vx;
      A(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
      A(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);
      A(3,0) = cos(yaw);
      A(4,0) = sin(yaw);
      A(3,1) = -sin(yaw);
      A(4,1) = cos(yaw);
      A(5,2) = 1;

      Aln = eye6 + (A*dt);
      Bln = B*dt;

      A_uio =(eye6 - E_diag*tau*C)*A*dt ;
      B_uio =(eye6 - E_diag*tau*C)*Bln;

    }

    void L_computation(float steer){

      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);

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

      MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

      for (int i = 0; i < 16; i++){

          result += mu_sch[i] * Llmi[i];

      }
      // depending on the formulation of LMI this sign may change 
      L = -result*dt;
    }
};

// ZKF UIO 

class ObserverLPV_UIOi{

  public:

    //handlers
    ros::NodeHandle n;

    //Clients
    ros::ServiceClient client = n.serviceClient<estimator::lmi_data>("LoadMatrices");
    estimator::lmi_data srv;

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

    //matrices
    MatrixXf C;
    MatrixXf A;
    MatrixXf B;
    MatrixXf Aln;
    MatrixXf Bln;
    MatrixXf A_uio;
    MatrixXf B_uio;
    MatrixXf L;
    MatrixXf E_diag = MatrixXf::Zero(n_states,n_states);
    MatrixXf Ew_diag = MatrixXf::Zero(n_states, n_states);
    MatrixXf Ev_diag = MatrixXf::Zero(n_outputs,n_outputs);

    //Vectors
    VectorXf Ew;
    VectorXf Ev;
    VectorXf E;
    VectorXf u;
    VectorXf y_ant = VectorXf::Zero(n_outputs);
    MatrixXf tau;
    MatrixXf Rxio = MatrixXf::Zero(n_states, n_states);
    VectorXf mu_sch;

    //DATA from matlab
    std::vector<MatrixXf> Llmi;
    std::vector<std::vector<float>> sched_vars;

    //estimated states
    float x      = 0.0;
    float y      = 0.0;
    float vx     = 1.0112;
    float vy     = 0.0115;
    float yaw    = 0.0;
    float psiDot = 0.0946;
    VectorXf states_est;

    //historic of values
    std::list <float> x_est_hist;
    std::list <float> y_est_hist;
    std::list <float> vx_est_hist;
    std::list <float> vy_est_hist;
    std::list <float> yaw_est_hist;
    std::list <float> psiDot_est_hist;
    std::list <MatrixXf> dis_est_max_hist;
    std::list <MatrixXf> dis_est_min_hist;
    std::list <std::vector<float>> upper_limits;
    std::list <std::vector<float>> lower_limits;

    void estimateState(GetSensorData sensor, GetActuData ecu){

      VectorXf y_meas;

      y_meas.resize(n_outputs);
      y_meas(0) = sensor.vx;
      y_meas(1) = sensor.psiDot;
      y_meas(2) = sensor.x;
      y_meas(3) = sensor.y;
      y_meas(4) = sensor.yaw;

      u(0)      = ecu.steer;
      u(1)      = ecu.a;

      // update matrices
      AB_computation(u(0));

      L_computation(u(0));

      //priori estimation

      // always noisiy estimation of the disturbance. 
      MatrixXf dis_est_max = tau*(y_meas - C*(Aln*states_est + Bln*u + Ew) - Ev);
      MatrixXf dis_est_min = tau*(y_meas - C*(Aln*states_est + Bln*u - Ew) + Ev);
      
      MatrixXf prior = states_est + A_uio*states_est + B_uio*u;

      states_est = prior + L * (y_meas - C*prior) + E_diag*tau*(y_meas - y_ant);

      //Expansion Matrices
      MatrixXf shape_priori;
      shape_priori.resize(int(Aln.rows()),int(Rxio.cols()+ n_states));

      shape_priori << Rxio + A_uio*Rxio, 2*(eye6 - E_diag*tau*C)*Ew_diag;

      Rxio.resize(int(n_states), shape_priori.cols() + n_outputs );

      MatrixXf aux_mat;
      aux_mat.resize(int(n_states), shape_priori.cols() + n_outputs);

      aux_mat << (eye6 - L*C)*shape_priori, (-E_diag*tau -L)*2*Ev_diag;

      Rxio = reduction(aux_mat,6);

      MatrixXf LimitMatrix = EnvBox(Rxio);

      std::vector<float> aux_vec_max;
      std::vector<float> aux_vec_min;

      for (int i = 0; i < n_states; i++){

        aux_vec_max.push_back(states_est(i) + fabs(LimitMatrix(i,i)));
        aux_vec_min.push_back(states_est(i) - fabs(LimitMatrix(i,i)));

      }

      upper_limits.push_back(aux_vec_max);
      lower_limits.push_back(aux_vec_min);

      // std::cout << states_est;
      //save states
      vx      = states_est(0);
      vy      = states_est(1);
      psiDot  = states_est(2);
      x       = states_est(3);
      y       = states_est(4);
      yaw     = states_est(5);

      y_ant = y_meas;

      //std::cout << dis_est(0,0) << std::endl;
      ROS_ERROR_STREAM("update finished: x " << x <<" y "<< y <<" yaw " << yaw <<" vx " << vx
              <<" vy "<< vy <<" yaw " << yaw <<" psiDot " << psiDot);

      /*std::cout << " dist " << dis_est_max(0) << std::endl;
      std::cout << " dist " << dis_est_min(0) << std::endl;*/

      x_est_hist.push_back(x);
      y_est_hist.push_back(y);
      yaw_est_hist.push_back(yaw);
      vx_est_hist.push_back(vx);
      vy_est_hist.push_back(vy);
      psiDot_est_hist.push_back(psiDot);
      dis_est_max_hist.push_back(dis_est_max);
      dis_est_min_hist.push_back(dis_est_min);
      
    }

    ObserverLPV_UIOi(){

      A.resize(n_states,n_states);
      B.resize(n_states,n_control);
      Aln.resize(n_states,n_states);
      Bln.resize(n_states,n_control);
      A_uio.resize(n_states,n_states);
      B_uio.resize(n_states,n_control);
      C.resize(n_outputs,n_states);
      L.resize(n_states,n_outputs);
      Ew.resize(n_states);
      E.resize(n_states);
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

      y_ant(0)      = vx;
      y_ant(1)      = psiDot;
      y_ant(2)      = x;
      y_ant(3)      = y;
      y_ant(4)      = yaw;


      C << 1, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0,
           0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 1;

      A << 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0;

      B << 0, 1,
           0, 0,
           0, 0,
           0, 0,
           0, 0,
           0, 0;

      u << 0,0;

      Ew << 0.01252/2,  0.0152, 0.1252, 0.004, 0.004, 0.008 ;
      Ev << 0.008   ,0.08   ,0.08  ,0.08  ,0.087;
      E  << -1/m  ,0.0   ,0.0  ,0.0  ,0.0 ,0.0 ;

      for (int i=0; i<n_states; i++){

        E_diag(i,i) = E(i);

      }

      for (int i=0; i<n_states; i++){

        Ew_diag(i,i) = Ew(i);

      }

      for (int i=0; i<n_outputs; i++){

        Ev_diag(i,i) = Ev(i);

      }

      MatrixXf aux_pinv = C*E_diag;
      tau = pseudoInverse(aux_pinv);


      states_est << vx, vy, psiDot, x, y, yaw;
      LoadMatrices();
    }

    private:

      int iterations = 0;

      void LoadMatrices(){

        srv.request.est_id = "ZKF";
        client.call(srv);

        ROS_ERROR_STREAM("resize v" << srv.response.L.size());
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

      //fill sched_vars

      sched_vars.resize(srv.response.limits.max.size());

      mu_sch.resize(srv.response.L.size());

      for (int i = 0; i < srv.response.limits.max.size(); i++){

        sched_vars[i].push_back(srv.response.limits.min[i]);
        sched_vars[i].push_back(srv.response.limits.max[i]);

      }

    }

    // void AB_computation(MatrixXf &A, MatrixXf &B, float theta, float steer, float vx, float vy){
    void AB_computation(float steer){

      //update B
      B(0,0) = -sin(steer) * Cf/m;
      B(1,0) = (cos(steer) * Cf) / m;
      B(2,0) = (lf * Cf * cos(steer)) / I;
      B(1,1) = cos(steer);
      B(2,1) = sin(steer);

      //Update A
      A(0,0) =  -mu;
      A(0,1) = (sin(steer) * Cf) / (m*vx);
      A(0,2) = ((sin(steer) * Cf * lf) / (m*vx)) + vy;
      A(1,1) = -(Cr + Cf * cos(steer)) / (m*vx);
      A(1,2) = -((lf * Cf * cos(steer) - lr * Cr) / (m*vx)) - vx;
      A(2,1) = -(lf * Cf * cos(steer) - lr * Cr) / (I*vx);
      A(2,2) = -(lf * lf * Cf * cos(steer) + lr * lr * Cr) / (I*vx);
      A(3,0) = cos(yaw);
      A(4,0) = sin(yaw);
      A(3,1) = -sin(yaw);
      A(4,1) = cos(yaw);
      A(5,2) = 1;

      Aln = eye6 + (A*dt);
      Bln = B*dt;

      A_uio =(eye6 - E_diag*tau*C)*A*dt;
      B_uio =(eye6 - E_diag*tau*C)*B*dt;

    }

    // void L_computation(MatrixXf &L, float vx, float vy, float theta, float steer){
    void L_computation(float steer){

      float M_vx_despl_min    = (sched_vars[0][1] - vx)    / (sched_vars[0][1] - sched_vars[0][0]);
      float M_vy_despl_min    = (sched_vars[1][1] - vy)    / (sched_vars[1][1] - sched_vars[1][0]);
      float M_steer_min       = (sched_vars[3][1] - steer) / (sched_vars[3][1] - sched_vars[3][0]);
      float M_theta_min       = (sched_vars[5][1] - yaw) / (sched_vars[5][1] - sched_vars[5][0]);

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

      MatrixXf result = MatrixXf::Zero(n_states,n_outputs);

      for (int i = 0; i < 16; i++){

          result += mu_sch[i] * Llmi[i];

      }

      L = -result*dt;
    }
};

//Overloaded class used to deal with errors, instead of this most of the times used matlab
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

    /*ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                      <<" vy "<< truth.vy <<" yaw " << truth.yaw <<" psiDot " << truth.psiDot);*/

    // std::cout << temp << '\n';
    errors.push_back(temp);
  }

  void compute_error(GetGroundTruthData truth,ObserverLPVi estimator){

    estimator::ErrorStates temp;

    temp.vx = truth.vx - estimator.vx;
    temp.vy = truth.vy - estimator.vy;
    temp.psiDot = truth.psiDot - estimator.psiDot;
    temp.x = truth.x - estimator.x;
    temp.y = truth.y - estimator.y;
    temp.psi = truth.yaw - estimator.yaw;

  /*  ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                      <<" vy "<< truth.vy <<" yaw " << truth.yaw <<" psiDot " << truth.psiDot);*/

    // std::cout << temp << '\n';
    errors.push_back(temp);
  }

  void compute_error(GetGroundTruthData truth,ObserverLPV_UIO estimator){

    estimator::ErrorStates temp;

    temp.vx = truth.vx - estimator.vx;
    temp.vy = truth.vy - estimator.vy;
    temp.psiDot = truth.psiDot - estimator.psiDot;
    temp.x = truth.x - estimator.x;
    temp.y = truth.y - estimator.y;
    temp.psi = truth.yaw - estimator.yaw;

  /*  ROS_ERROR_STREAM("Ground Truth: x " << truth.x <<" y "<< truth.y <<" yaw " << truth.yaw <<" vx " << truth.vx
                      <<" vy "<< truth.vy <<" yaw " << truth.yaw <<" psiDot " << truth.psiDot);*/

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

    std::cout << truth.x_hist.size() << " " << estimator.x_est_hist.size() << std::endl;

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

  void save_error(GetGroundTruthData truth,ObserverLPV_it estimator, std::string folder){

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

    void save_error(GetGroundTruthData truth,ObserverLPVi estimator, std::string folder){

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

    msg.folder = folder;
    msg_data.folder = folder;

    error_pub.publish(msg);
    data_pub.publish(msg_data);

  }

    void save_error(GetGroundTruthData truth,ObserverLPV_UIO estimator, std::string folder){

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

      aux.x       = estimator.dis_est_max_hist.front()(3);
      aux.y       = estimator.dis_est_max_hist.front()(4);
      aux.psi     = estimator.dis_est_max_hist.front()(5);
      aux.vx      = estimator.dis_est_max_hist.front()(0);
      aux.vy      = estimator.dis_est_max_hist.front()(1);
      aux.psiDot  = estimator.dis_est_max_hist.front()(2);

      estimator.dis_est_max_hist.pop_front();
      msg_data.max_dist.push_back(aux);

      aux.x       = estimator.dis_est_min_hist.front()(3);
      aux.y       = estimator.dis_est_min_hist.front()(4);
      aux.psi     = estimator.dis_est_min_hist.front()(5);
      aux.vx      = estimator.dis_est_min_hist.front()(0);
      aux.vy      = estimator.dis_est_min_hist.front()(1);
      aux.psiDot  = estimator.dis_est_min_hist.front()(2);

      estimator.dis_est_min_hist.pop_front();
      msg_data.min_dist.push_back(aux);

    }

    msg.folder = folder;
    msg_data.folder = folder;

    error_pub.publish(msg);
    data_pub.publish(msg_data);

  }

   void save_error(GetGroundTruthData truth,ObserverLPV_UIOi estimator, std::string folder){

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

      aux.x       = estimator.dis_est_max_hist.front()(3);
      aux.y       = estimator.dis_est_max_hist.front()(4);
      aux.psi     = estimator.dis_est_max_hist.front()(5);
      aux.vx      = estimator.dis_est_max_hist.front()(0);
      aux.vy      = estimator.dis_est_max_hist.front()(1);
      aux.psiDot  = estimator.dis_est_max_hist.front()(2);

      estimator.dis_est_max_hist.pop_front();
      msg_data.max_dist.push_back(aux);

      aux.x       = estimator.dis_est_min_hist.front()(3);
      aux.y       = estimator.dis_est_min_hist.front()(4);
      aux.psi     = estimator.dis_est_min_hist.front()(5);
      aux.vx      = estimator.dis_est_min_hist.front()(0);
      aux.vy      = estimator.dis_est_min_hist.front()(1);
      aux.psiDot  = estimator.dis_est_min_hist.front()(2);

      estimator.dis_est_min_hist.pop_front();
      msg_data.min_dist.push_back(aux);

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

    msg.folder = folder;
    msg_data.folder = folder;

    error_pub.publish(msg);
    data_pub.publish(msg_data);

  }

};

