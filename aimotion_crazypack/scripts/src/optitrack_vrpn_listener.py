#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Publisher received: \n%s", data.pose.position)

def listener():
    rospy.init_node("optitrack_vrpn_listener", anonymous = True)
    rospy.Subscriber("/vrpn_client_node/object_0/pose", PoseStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    # listener()
    rospy.init_node("optitrack_vrpn_listener", anonymous = True)
    rospy.Subscriber("/vrpn_client_node/object_0/pose", PoseStamped)
    res = rospy.wait_for_message(topic = '/vrpn_client_node/object_0/pose', topic_type = PoseStamped, timeout = None)
    print("Message received! :)")
    print(res.pose.position)
