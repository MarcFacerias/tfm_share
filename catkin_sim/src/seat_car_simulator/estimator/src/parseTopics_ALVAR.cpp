#define _GLIBCXX_USE_C99 1
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <sstream>
#include <deque>
#include <string>
#include <sstream>
#include <mutex>          // std::mutex

/*

Code used to detected landmarks using alvar markers, discarted due to problems with temporal aligment
but should work if the model of the car is simulated in gazebo, may require updates 

*/

// GLOBAL VARS
std::deque<geometry_msgs::Pose> pose_landmark;
std::deque<int>  id_landmark;
bool publish_flag = true;
std::mutex mtx;           // mutex for critical section

// Alternativa


template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

void GetPoseCallbackC1(const ar_track_alvar_msgs::AlvarMarkers& msg)
{

  if (!msg.markers.empty()){

    for (int j = 0; j< msg.markers.size(); j++){

      mtx.lock();
      pose_landmark.push_back(msg.markers[j].pose.pose);
      id_landmark.push_back(msg.markers[j].id);
      pose_landmark.back().position.x = msg.markers[j].pose.pose.position.y;
      pose_landmark.back().position.y = msg.markers[j].pose.pose.position.x;
      mtx.unlock();

    }
  }
}

void GetPoseCallbackC2(const ar_track_alvar_msgs::AlvarMarkers& msg)
{

  if (!msg.markers.empty()){

    for (int j = 0; j< msg.markers.size(); j++){

      mtx.lock();
      pose_landmark.push_back(msg.markers[j].pose.pose);
      id_landmark.push_back(msg.markers[j].id);
      pose_landmark.back().position.x = -msg.markers[j].pose.pose.position.y;
      pose_landmark.back().position.y = -msg.markers[j].pose.pose.position.x;
      mtx.unlock();

    }
  }
}

void GetPoseCallbackC3(const ar_track_alvar_msgs::AlvarMarkers& msg)
{

  if (!msg.markers.empty()){

    for (int j = 0; j< msg.markers.size(); j++){

      mtx.lock();
      pose_landmark.push_back(msg.markers[j].pose.pose);
      id_landmark.push_back(msg.markers[j].id);
      pose_landmark.back().position.x = -msg.markers[j].pose.pose.position.x;
      mtx.unlock();

    }
  }
}

void GetPoseCallbackC4(const ar_track_alvar_msgs::AlvarMarkers& msg)
{

  if (!msg.markers.empty()){

    for (int j = 0; j< msg.markers.size(); j++){

      pose_landmark.push_back(msg.markers[j].pose.pose);
      id_landmark.push_back(msg.markers[j].id);
      pose_landmark.back().position.y = -msg.markers[j].pose.pose.position.y;
      mtx.unlock();

    }
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ParseMessage");

  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  ros::Publisher pose_pub = n.advertise<ar_track_alvar_msgs::AlvarMarkers>("/Corrected_Pose", 1);

  ros::Subscriber subC1 = n.subscribe("/camera1_alvar", 1, GetPoseCallbackC1);
  ros::Subscriber subC2 = n.subscribe("/camera2_alvar", 1, GetPoseCallbackC2);
  ros::Subscriber subC3 = n.subscribe("/camera3_alvar", 1, GetPoseCallbackC3);
  ros::Subscriber subC4 = n.subscribe("/camera4_alvar", 1, GetPoseCallbackC4);


  tf::TransformListener listener_;

  while (ros::ok())
  {

    if (!pose_landmark.empty()){

        ar_track_alvar_msgs::AlvarMarkers msg_lnd_corrected;
        ar_track_alvar_msgs::AlvarMarker lnd_corrected;

        for (int i = 0; i < pose_landmark.size(); i++ ){

            std::stringstream ss_tag;
            std::stringstream ss_landmark;
            std::string str;
            str = ToString(id_landmark[i]);

            ss_tag << "landmark" << str[0] << "_tag_image" << str[1];
            ss_landmark << "landmark" << str[0] << "_link";


            tf::StampedTransform tag_center;
            tf::Transform cam_center;
            tf::Transform cam_tag;

            tf::poseMsgToTF(pose_landmark[i], cam_tag);

            try{
                 listener_.lookupTransform(  ss_landmark.str(), ss_tag.str(), ros::Time(0), tag_center);
                 // listener_.lookupTransform(  ss_tag.str(),ss_landmark.str(), ros::Time(0), tag_center);
            }
            catch (tf::TransformException &ex) {
              ROS_WARN("%s",ex.what());
              ros::Duration(1.0).sleep();
              continue;
            }
            tag_center.setRotation( tf::createQuaternionFromRPY(0,0,0) );
            cam_tag.setRotation( tf::createQuaternionFromRPY(0,0,0) );

            cam_center = cam_tag * tag_center;
            tf::poseTFToMsg(cam_center, lnd_corrected.pose.pose);

            lnd_corrected.id = str[0] - '0';
            msg_lnd_corrected.markers.push_back(lnd_corrected);
            pose_landmark.pop_front();
            id_landmark.pop_front();

        }

        pose_pub.publish(msg_lnd_corrected);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
