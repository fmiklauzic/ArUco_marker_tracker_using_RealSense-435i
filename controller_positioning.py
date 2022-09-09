#!/usr/bin/env python  

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import tanh
import tf2_ros
import time 
import shlex
from psutil import Popen

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
sub = rospy.Subscriber("/odom_encoder", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()
r = rospy.Rate(30)
time.sleep(3)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

goal = Point()
#goal.x = 0
#goal.y = 0
priv_goal = Point()
priv_goal.x = 0
priv_goal.y = 0
coeff_angle = 0.5
coeff_linear = 0.3

while not rospy.is_shutdown():
	try:
        	trans = tfBuffer.lookup_transform('robot_link', 'novi_koordinatni', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		speed.linear.x = 0.0
		speed.angular.z = 0.2*coeff_angle
		pub.publish(speed) 
        	r.sleep()
		
        	continue
		
	goal.x = trans.transform.translation.x
	goal.y = trans.transform.translation.y
	

	
	
	inc_x = goal.x - x
	inc_y = goal.y - y

	angle_to_goal = atan2(inc_y, inc_x)
	
	rospy.loginfo('x: {}, y:{}'.format(round(inc_x,3),round(inc_y,3) ))
	rospy.loginfo('angle_to_goal: {}, theta:{}'.format(round(angle_to_goal,3),round(theta,3) ))
	

	angle_dif = angle_to_goal - theta
	if inc_x < 0.02:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		try:
        		trans2 = tfBuffer.lookup_transform('robot_link', 'aruco_marker_frame', rospy.Time())
        	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			if goal.y < 0:
				speed.linear.x = 0.0
				speed.angular.z = 0.2*coeff_angle
			else:
				speed.linear.x = 0.0
				speed.angular.z = -0.2*coeff_angle
				
			pub.publish(speed) 
        		r.sleep()
        		continue
		rotx = trans2.transform.rotation.x
		roty = trans2.transform.rotation.y
		rotz = trans2.transform.rotation.z
		rotw = trans2.transform.rotation.w
		(roll_n, pitch_n, theta_n) = euler_from_quaternion([rotx, roty, rotz, rotw])
		rospy.loginfo('roll: {}'.format(round(roll_n,5) ))
		rospy.loginfo('pitch: {}'.format(round(pitch_n,5) ))
		rospy.loginfo('theta: {}'.format(round(theta_n,5) ))
		#rospy.loginfo('rotacija: {}'.format(round(pitch_n,5) ))
		
		if roll_n > 0.05:
			speed.linear.x = 0.0
			speed.angular.z = 0.05*coeff_angle
		elif roll_n < -0.05:
			speed.linear.x = 0.0
			speed.angular.z = -0.05*coeff_angle
		else:
			speed.linear.x = 0.0
			speed.angular.z =0.0

	elif abs(angle_dif) < 0.04 and inc_x > 1.5 :
		speed.linear.x = coeff_linear*1.5
		speed.angular.z = 0

	elif abs(angle_dif) > 0.1:
		speed.linear.x = 0
		speed.angular.z = coeff_angle*tanh(angle_dif)
	elif angle_dif <= 0.1 and angle_dif >= 0.04 and inc_x >= 0.2:
		speed.linear.x = coeff_linear*inc_x
		speed.angular.z = coeff_angle*0.02
	elif angle_dif <= -0.1 and angle_dif >= -0.04 and inc_x >= 0.2:
		speed.linear.x = coeff_linear*inc_x
		speed.angular.z = coeff_angle*(-0.02)


	elif abs(angle_dif) < 0.04 and inc_x < 1.5 and inc_x > 0.2 :
		speed.linear.x = coeff_linear*inc_x
		speed.angular.z = 0
	elif abs(angle_dif) < 0.04 and inc_x <= 0.2 and inc_x > 0.02 :
		speed.linear.x = coeff_linear*(inc_x+0.1)*0.8
		speed.angular.z = 0
	#elif abs(angle_dif) < 0.04 and inc_x <= 0.1 :
	#	speed.linear.x = coeff_linear*0.02
	#	speed.angular.z = 0

	elif angle_dif <= 0.1 and angle_dif >= 0.04 and inc_x <= 0.2 and inc_x > 0.02:
		speed.linear.x = coeff_linear*(inc_x+0.1)*0.5
		speed.angular.z = coeff_angle*0.02
	#elif angle_dif <= 0.1 and angle_dif >= 0.04 and inc_x <= 0.1 :
	#	speed.linear.x = coeff_linear*0.02
	#	speed.angular.z = coeff_angle*0.02

	elif inc_x < -0.1:
		speed.linear.x = coeff_linear*(inc_x)	
		speed.angular.z = 0.0
	else:
		speed.linear.x = 0.0
		speed.angular.z = 0.0


	pub.publish(speed)
	r.sleep()    
