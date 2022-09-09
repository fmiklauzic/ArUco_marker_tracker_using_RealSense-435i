#!/usr/bin/env python  

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import tanh
import tf2_ros
import time 

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")
#sub = rospy.Subscriber("/odometry", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()
r = rospy.Rate(25)
time.sleep(3)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

goal = Point()
priv_goal = Point()
priv_goal.x = 0
priv_goal.y = 0
coeff_angle = 1.4
coeff_linear = 0.23

while not rospy.is_shutdown():
	try:
        	trans = tfBuffer.lookup_transform('robot_link', 'aruco_marker_frame', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		speed.linear.x = 0.0
		speed.angular.z = 0.2*coeff_angle
		pub.publish(speed) 
        	r.sleep()
        	continue
		
	goal.x = trans.transform.translation.x
	goal.y = trans.transform.translation.y

	


	goal.x = trans.transform.translation.x
	goal.y = trans.transform.translation.y
	rotx = trans.transform.rotation.x
	roty = trans.transform.rotation.y
	rotz = trans.transform.rotation.z
	rotw = trans.transform.rotation.w

	(roll_n, pitch_n, theta_n) = euler_from_quaternion([rotx, roty, rotz, rotw])
	
	
	
	inc_x = goal.x - x
	inc_y = goal.y - y

	angle_to_goal = atan2(inc_y, inc_x)
	
	rospy.loginfo('x: {}, y:{}'.format(round(inc_x,3),round(inc_y,3) ))
	rospy.loginfo('angle_to_goal: {}, theta:{}'.format(round(angle_to_goal,3),round(theta,3) ))
	rospy.loginfo('rotacija: {}'.format(round(theta_n,5) ))
	#rospy.loginfo('pozicija: {}'.format(round(inc_x,15) ))

	angle_dif = angle_to_goal - theta
	if inc_x < 0.8 and inc_x >= 0.7:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
	elif inc_x < 0.7:
		speed.linear.x = -coeff_linear*(0.75-inc_x)*1.5
		speed.angular.z = 0.0
	elif abs(angle_dif) > 0.1:
		speed.linear.x = inc_x*coeff_linear*(inc_x-0.7)
		speed.angular.z = coeff_angle*tanh(angle_dif)
	elif angle_dif <= 0.1 and inc_x >= 0.8 and inc_x <= 1.5:
		speed.linear.x = inc_x*coeff_linear*(inc_x-0.7)
		speed.angular.z = coeff_angle*0.01
	elif angle_dif >= -0.1 and inc_x >= 0.8 and inc_x <= 1.5:
		speed.linear.x = inc_x*coeff_linear*(inc_x-0.7)
		speed.angular.z = coeff_angle*(-0.01)
	elif inc_x > 1.5:
		speed.linear.x = coeff_linear*1.1
		speed.angular.z = 0.0
	elif inc_x <= 1.5 and inc_x >= 0.8:
		speed.linear.x = coeff_linear*(inc_x-0.6)*1.1
		speed.angular.z = 0.0
	#elif inc_x < 0.7 and inc_x >= 0.5:
	#	speed.linear.x = coeff_linear*inc_x
	#	speed.angular.z = 0.0
	else:
		speed.linear.x = 0.0
		speed.angular.z = 0.0

	pub.publish(speed)
	r.sleep()    
