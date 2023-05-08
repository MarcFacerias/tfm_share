#define _GLIBCXX_USE_C99 1
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "tf/transform_datatypes.h"
#include <lpv_mpc/simulatorStates.h>
#include <sstream>
#include <deque>
#include <string>
#include <sstream>

/*

Compute the pose of each landmark wrt the car using the tf package, the sensor limit 
is considered to be in a circle of radius 2m, best approach would be to simulate using 
some sensor model in gazebo -> need to implement car dynamics there + some low level control

*/


// GLOBAL VARS
std::deque<geometry_msgs::Pose> pose_landmark;
std::deque<int>  id_landmark;
bool publish_flag = true;

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

void GetPoseCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{

  if (!msg.markers.empty()){

    for (int j = 0; j< msg.markers.size(); j++){

      pose_landmark.push_back(msg.markers[j].pose.pose);
      id_landmark.push_back(msg.markers[j].id);

    }
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ParseMessage");

  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  ros::Publisher pose_pub = n.advertise<ar_track_alvar_msgs::AlvarMarkers>("/Corrected_Pose", 1);

  tf::TransformListener listener_;

  double roll, pitch, yaw;

  while (ros::ok())
  {

        ar_track_alvar_msgs::AlvarMarkers msg_lnd_corrected;
        ar_track_alvar_msgs::AlvarMarker lnd_corrected;

        for (int i = 1; i < 8; i++ ){

            std::stringstream ss_tag;
            std::stringstream ss_landmark;
            std::string str;
            str = ToString(i);

              ss_landmark << "landmark" << str[0] << "_link";

              tf::StampedTransform tag_center;
              
              tf::Transform cam_center;
              tf::Transform cam_tag;

              geometry_msgs::PoseStamped robot;
              tf::StampedTransform robot_center;
              
              try{

                   listener_.lookupTransform(  "base_link", ss_landmark.str(), ros::Time(0), tag_center);

              }

              catch (tf::TransformException &ex) {

                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;

              }

              try{

               listener_.lookupTransform(  "world", "base_link", ros::Time(0), robot_center);

              }

              catch (tf::TransformException &ex) {

                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
                
              }

              tf::poseTFToMsg(robot_center, robot.pose);
              tf::Quaternion quat;
              tf::quaternionMsgToTF(robot.pose.orientation, quat);
              tf::Matrix3x3 m(quat);
              m.getRPY(roll, pitch, yaw);

              tag_center.setRotation( tf::createQuaternionFromRPY(0,0,0));

              cam_center = tag_center;
              tf::poseTFToMsg(cam_center, lnd_corrected.pose.pose);
              

              if (sqrt((lnd_corrected.pose.pose.position.x) * (lnd_corrected.pose.pose.position.x) 
                    +  (lnd_corrected.pose.pose.position.y) * (lnd_corrected.pose.pose.position.y)) < 2.0)

              {

                  lnd_corrected.id = str[0] - '0';
                  float aux_y = lnd_corrected.pose.pose.position.y;
                  float aux_x = lnd_corrected.pose.pose.position.x;

                  lnd_corrected.pose.pose.position.x = aux_x;
                  lnd_corrected.pose.pose.position.y = -aux_y;

                  lnd_corrected.pose.pose.orientation.x = robot.pose.position.x;
                  lnd_corrected.pose.pose.orientation.y = -robot.pose.position.y;
                  lnd_corrected.pose.pose.orientation.z = -yaw;

                  msg_lnd_corrected.markers.push_back(lnd_corrected);

              }

        }

        pose_pub.publish(msg_lnd_corrected);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
