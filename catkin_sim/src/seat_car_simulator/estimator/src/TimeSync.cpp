#define _GLIBCXX_USE_C99 1
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <geometry_msgs/PoseStamped.h>
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
#include <estimator/LandmarkInfo.h>
#include <estimator/LandmarksInfo.h>
#include <estimator/DataInfo.h>
#include <iostream>
#include <chrono>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <unistd.h>
#include<Eigen/Core>
#include<Eigen/SVD>

using namespace std;

/*

Code used to associate data between ROS simulator and Gazebo platform, 
there's a delay associated with the fact that gazebo takes
some time updating the position of the car + introduces uncontrolled 
noise into the system (some drift that makes the system unusable)

*/

deque<vector<float>> simData;  //it,vx,vy,w,x,y,yaw
deque<vector<float>> sensData; //it,vx,vy,w,x,y,yaw
deque<vector<float>> actuData; //it, motor, servo
deque<vector<float>> landData; //id, x,y,x_m,y_m

float it_sim  = 0;
float it_sens = 0;
float it_actu = 0;
float it_land = 0;
float pos_related =0;
float start_measure =0;

ros::Publisher actu_pub;
ros::Publisher sim_pub;
ros::Publisher pose_pub;
ros::Publisher land_pub;
ros::Publisher flag_pub;

float x_gazebo = 0;
float y_gazebo = 0;
float yaw_gazebo = 0;

bool flag = true;
bool flag_actuPub = false;
bool flag_firstActu = false;
bool flag_actu_lnd = false;

ar_track_alvar_msgs::AlvarMarkers lnd_msg;

float wrap(double deltaPhase)
{
  do{

    if (deltaPhase>0)
        deltaPhase = fmod(deltaPhase+M_PI, 2.0*M_PI)-M_PI;
    else
        deltaPhase = fmod(deltaPhase-M_PI, 2.0*M_PI)+M_PI;

  }while ((deltaPhase < -M_PI) || (deltaPhase > M_PI));


    return deltaPhase;
}



void GetActuCallback(const lpv_mpc::ECU& msg)
{

    it_actu++;
    //cout <<"Hi" << endl;
    flag_firstActu = true;
    vector<float> aux{it_actu,msg.motor,msg.servo, it_sim, 0 };
    actuData.push_back(aux);

}

void GetGroundTruthDataCallback(const l4vehicle_msgs::VehicleState& msg)
{

    it_sim++;
    vector<float> aux{it_sim,msg.longitudinal_velocity ,msg.lateral_velocity,msg.angular_velocity,msg.x,msg.y,msg.heading}; 
    simData.push_back(aux);

}

void GetPoseCallback(const lpv_mpc::simulatorStates& msg)
{

    it_sens++;
    vector<float> aux{it_sens,msg.vx,msg.psiDot,msg.x,msg.y,msg.psi}; 
    sensData.push_back(aux);

} 

void GetMarkersCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{   

    it_land++;

    lnd_msg = msg;

    for (int i = 0; i < msg.markers.size(); i++){

       x_gazebo = msg.markers[i].pose.pose.orientation.x;
       y_gazebo = msg.markers[i].pose.pose.orientation.y;
       yaw_gazebo = msg.markers[i].pose.pose.orientation.z;
       flag_actu_lnd = true;

  }

  flag_actu_lnd = true;

}

void SyncMsg(){

  if (simData.back()[1] > 1.0){

    start_measure++;

    int it = 0;
    bool send_msg = false;
    float d_ant = 1000;
    float d = 100;
    lpv_mpc::simulatorStates msg_sen;
    l4vehicle_msgs::VehicleState msg_sim;
    lpv_mpc::ECU msg_actu;


     do{

      //cout << "acces sim data" << endl;
      d = sqrt(pow(x_gazebo-simData[it][4],2) + pow(y_gazebo-simData[it][5],2));

      if (d < 0.02){

        send_msg = true;
        pos_related++;
        break;

      }
       
      if  (it < simData.size()-1){

        it++;

      }else{

        break;

      }
      

    }while(1);

    //cout << "acces sim data" << endl;
    if (send_msg){

        msg_sim.longitudinal_velocity         =   float(simData[it][1]);
        msg_sim.lateral_velocity              =   float(simData[it][2]);      
        msg_sim.angular_velocity              =   float(simData[it][3]);
        msg_sim.x                             =   float(simData[it][4]);
        msg_sim.y                             =   float(simData[it][5]);
        msg_sim.heading                       =   float(simData[it][6]);

        sim_pub.publish(msg_sim);

        if (flag_actu_lnd){

          for (int i = 0; i < lnd_msg .markers.size(); i++){

            float aux_x = lnd_msg.markers[i].pose.pose.position.x;
            float aux_y = lnd_msg.markers[i].pose.pose.position.y;

            float dx    = (x_gazebo   - msg_sim.x);
            float dy    = (y_gazebo   - msg_sim.y);
            float dyaw  = ((yaw_gazebo) - (msg_sim.heading));

            lnd_msg.markers[i].pose.pose.position.x = dx + cos(dyaw)*aux_x - sin(dyaw)*aux_y;
            lnd_msg.markers[i].pose.pose.position.y = dy + sin(dyaw)*aux_x + cos(dyaw)*aux_y;

        }

          flag_actu_lnd = false;
          land_pub.publish(lnd_msg);

        }

        cout << msg_sim.x << " " << msg_sim.y << " " <<  wrap(msg_sim.heading) << " "<< 
             x_gazebo << " " << y_gazebo << " " << wrap(yaw_gazebo) << endl;

        if (flag){
          std_msgs:: Bool flag_start;
          flag_start.data = true;
          flag_pub.publish(flag_start);
          flag = false;
        }


        int sens_it = 0;

        //cout << "acces sens data" << endl;
        while (sens_it < sensData.size()){

          if (sensData[sens_it][0] == simData[it][0]){

              msg_sen.vx                          =   sensData[sens_it][1];
              msg_sen.psiDot                      =   sensData[sens_it][2];
              msg_sen.x                           =   sensData[sens_it][3];
              msg_sen.y                           =   sensData[sens_it][4];
              msg_sen.psi                         =   sensData[sens_it][5];
 
              pose_pub.publish(msg_sen);

              break;

          }

          else if (sens_it +1 >= sensData.size()){
            break;
          }

          else{

            sens_it ++;

          }
        }

        int actu_it = 0;

        //cout << "acces actu data" << endl;
        while (actu_it < actuData.size() && !actuData.empty()){

          if (  simData[it][0] >= actuData[actu_it][3]){

              msg_actu.motor    =   actuData[actu_it][1];
              msg_actu.servo    =   actuData[actu_it][2];
              actu_pub.publish(msg_actu);
              flag_actuPub = true;
              break;

          }

          else if (actu_it + 1 >= actuData.size()){
            break;
          }

          else{

            actu_it ++;

          }
        }  
        int j = 0;

        //cout << "acces sim data" << endl;
        while (j <= it){

          simData.pop_front();
          j++;

        }

        //cout << "acces actu data" << endl;
        if (flag_actuPub){

            j = 0;
            while (j <= actu_it){

              actuData.pop_front();
              j++;

            }
            flag_actuPub = false;

        }
        j = 0;

        //cout << "acces sens data" << endl;
        while (j <= sens_it){

          sensData.pop_front();
          j++;

        }
    }
    send_msg = false;
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Sync");

  ros::WallTime start_, end_;

  ros::NodeHandle n;

  ros::Rate loop_rate(200);

  tf::TransformListener listener_;

  ros::Subscriber actu_sub = n.subscribe("/ecu", 1, GetActuCallback); 
  ros::Subscriber sim_sub = n.subscribe("/vehicle_state", 1, GetGroundTruthDataCallback);
  ros::Subscriber pose_sub = n.subscribe("/sensorStates", 1, GetPoseCallback);
  ros::Subscriber land_sub = n.subscribe("/Corrected_Pose", 1, GetMarkersCallback);

  actu_pub = n.advertise<lpv_mpc::ECU>("/ecu_s", 1); 
  sim_pub  = n.advertise<l4vehicle_msgs::VehicleState>("/vehicle_state_s", 1); 
  pose_pub = n.advertise<lpv_mpc::simulatorStates>("/sensorStates_s", 1); 
  land_pub = n.advertise<ar_track_alvar_msgs::AlvarMarkers>("/Corrected_Pose_s", 1); 
  flag_pub = n.advertise<std_msgs::Bool>("SyncFlag", 1);


  float vx,vy,w,x,y,yaw;

  geometry_msgs::PoseStamped robot;
  tf::StampedTransform robot_center;


  listener_.waitForTransform("/world", "/base_link",
                              ros::Time(0),ros::Duration(3.0));

  while (ros::ok())
  {
        start_ = ros::WallTime::now();
        try{
         //listener_.lookupTransform(  ss_landmark.str(), ss_tag.str(), ros::Time(0), tag_center);
         listener_.lookupTransform("world","base_link",ros::Time(0), robot_center);
        }
        catch (tf::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        tf::poseTFToMsg(robot_center, robot.pose);

        x_gazebo = robot.pose.position.x;
        y_gazebo = -robot.pose.position.y;

        //cout << "hi " << endl;

        if (!simData.empty() && flag_firstActu && !sensData.empty() ){

          //cout << "hi 2" << endl;

          //cout << simData.empty() << actuData.empty() << endl;

          SyncMsg();


          //cout << "data analised" << simData.size() << " " << actuData.size() << " " << sensData.size() <<endl;

        }

        else{

          cout<< "not enough data" << endl;
        }
        std_msgs::Bool flag_start;
        flag_start.data = true;
        flag_pub.publish(flag_start);
        end_ = ros::WallTime::now();
        double execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
        ros::spinOnce();
        loop_rate.sleep();
    }


  cout << pos_related/start_measure << endl;
  return 0;
}
