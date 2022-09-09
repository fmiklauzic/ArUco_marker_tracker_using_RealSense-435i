#!/usr/bin/env python  

import rospy
import tf2_ros
import geometry_msgs.msg
import math

if __name__ == '__main__':
    rospy.init_node('adding_frame_robot_link')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "robot_link"
    t.child_frame_id = "camera_link"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        x = rospy.Time.now().to_sec() * math.pi

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = 0.14
        t.transform.translation.y = 0.035
        t.transform.translation.z = 0.05
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = -0.5

        br.sendTransform(t)
        rate.sleep()
