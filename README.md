# ArUco_marker_tracker_using_RealSense-435i
Tracking of ArUco markers using ROS, mobile robot and RealSense camera

Filip Miklaužić
-Object tracking with a mobile robot using computer vision
---------------------------------------------------------------
Faculty of Mechanical Engineering and Naval Architecture, Zagreb



---------------------------------------------------------------
This repository was made as an addition to the university final thesis.
Link:

All python scripts from this repository can be used in independent projects with minor changes
Launch files that are attached are made for the specific case, but can be easily customized
---------------------------------------------------------------
System:
ROS Melodic Morenia
Ubuntu 18.04
Python 2.7
---------------------------------------------------------------
Required packages:
https://github.com/pal-robotics/ddynamic_reconfigure
https://github.com/IntelRealSense/realsense-ros.git
https://github.com/pal-robotics/aruco_ros
---------------------------------------------------------------
///////////////////////////////////////////////////////////////
---------------------------------------------------------------

---------------------------------------------------------------
-add_robot_link.py:
Script that are made for creating a static transformation between camera and robot frame
---------------------------------------------------------------

---------------------------------------------------------------
-static_trans_marker:
Script that are made for creating a point in space that the mobile robot must reach
---------------------------------------------------------------

---------------------------------------------------------------
-controller_positioning:
Script that are made for the task of positioning in relation to the ArUco marker
---------------------------------------------------------------

---------------------------------------------------------------
-controller_following:
Script that are made for following ArUco marker
---------------------------------------------------------------

---------------------------------------------------------------
-marker_positioning.launch:
Script made for launch nodes for positioning in relation to the ArUco marker
---------------------------------------------------------------

---------------------------------------------------------------
-follow_marker.launch:
Script made for launch nodes for following ArUco marker
---------------------------------------------------------------

---------------------------------------------------------------
-find_object_2d.launch:
Script made to launch nodes for detecting objects on the scene using RGB picture
---------------------------------------------------------------

---------------------------------------------------------------
-find_object_3d.launch:
Script made to launch nodes for detecting objects on the scene using RGB and PCL data
---------------------------------------------------------------

---------------------------------------------------------------
rviz_temp_aruco_2.rviz:
Template for graphical display of active topics
---------------------------------------------------------------
