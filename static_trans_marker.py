#!/usr/bin/env python  

import roslib
import rospy
import tf
from math import pi
import shlex
from psutil import Popen


def getTransform():
	listener = tf.TransformListener()
	listener.waitForTransform("/aruco_marker_frame", "/robot_link", rospy.Time(), rospy.Duration(100.0))
	(trans, rot) = listener.lookupTransform("/aruco_marker_frame", "/robot_link",rospy.Time())
	node_process = Popen(shlex.split('rosservice call /resetEncoderOdom'))
	return trans, rot

if __name__ == '__main__':
	rospy.init_node('static_tf_broadcaster')
	trans, rot = getTransform()
	transl_Filip = (trans[0] , trans[1], trans[2] - 0.500)
	rate = rospy.Rate(100)
	
    	while not rospy.is_shutdown():
		br = tf.TransformBroadcaster()
		br.sendTransform(transl_Filip, rot, rospy.Time.now(), "/robot_link", "/novi_koordinatni")
		rate.sleep()


