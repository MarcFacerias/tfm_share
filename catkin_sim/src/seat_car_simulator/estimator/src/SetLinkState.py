#!/usr/bin/env python
"""
    File name: Learning-LPV-MPC.py
    Author: Eugenio Alcala
    Email: eugenio.alcala@upc.edu.edu
    Date: 09/30/2018
    Python Version: 2.7.12
"""
from gazebo_msgs.msg import LinkState, ModelState
import rospy

def main():

    rospy.init_node("SetLink")
    r = rospy.Rate(200)
    rospy.sleep(1)
    # pub_linkStates = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=1)
    pub_linkStates = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    # link_msg = LinkState()
    link_msg = ModelState()

    # link_msg.link_name = "base_link"
    link_msg.model_name = "seat_car"
    link_msg.reference_frame = "world"

    link_msg.pose.position.x = 0
    link_msg.pose.position.y = 0
    link_msg.pose.position.z = 0.031

    link_msg.pose.orientation.x = 0
    link_msg.pose.orientation.y = 0
    link_msg.pose.orientation.z =  0
    link_msg.pose.orientation.w =  1

    link_msg.twist.linear.x = 0.1
    link_msg.twist.linear.y = 0
    link_msg.twist.linear.z = 0

    link_msg.twist.angular.x = 0.0
    link_msg.twist.angular.y = 0
    link_msg.twist.angular.z = 0

    while not pub_linkStates.get_num_connections() > 0:
        pass
    while not rospy.is_shutdown():
        link_msg.pose.position.x = link_msg.pose.position.x + 0.005

        pub_linkStates.publish(link_msg)
        r.sleep()

    quit()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass