--------------------------------------------------------------------------------
Lecture: TAS WS18/19
Contribution: Slalom
Group: 5
Student: Michael Wachl
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
# Task description:
--------------------------------------------------------------------------------
Drive a trajectory through 4 cones (left, right, ...).
Trajectory shoud be ajusted to enviromental information.

--------------------------------------------------------------------------------
# Main idea:
--------------------------------------------------------------------------------
My main idea is to set different goal points. Each goal point is set 
next to a cone with same orientation. The last goal point is set behind the last 
cone. This can be achieved with the actionlib (MoveBaseClient). Teb local planner 
can then be used to plan the trajectory, recover if trapped and avoid collision 
with a cone. 

--------------------------------------------------------------------------------
# Implementation:
--------------------------------------------------------------------------------
I implemented two solutions for the task.
I started with a primitiv goal publishing:
- get initial position and orietation from rviz (set it there)
- calculate all goal points with the start point orientation and fixed distances
- then publish them with the MoveBaseClient and a trajectory queue
- the implemented local planner is then responsible for a smooth trajectory
This resulted very high dependency in the initial position and a possible 
failiure. 

I decided to implement a advanced solution:
- subscribe to laser scan and convert it into a pointcloud
- do filtering in x (front) and y (side) dimention and cluster the cloud
- after filtering the only cluster left is the next cone (angle is checked)
- calculate centroid and then transform the position into map koordinates
- then calculate the next goal coordinates accordingly to the detected cone
  coordinates and the orientation of previous and detected cone
- publish goal by goal with MovebaseClient
This solution takes all the position of the cones into account. Therefore it 
doesn't matter if the inital position or the position of the cones are not 
perfect. 

--------------------------------------------------------------------------------
# Problems:
--------------------------------------------------------------------------------
- The teb local planner doesen't drive a smooth trajectory and adjusts its path 
frequently, after final testing menkog performed better than amur. 
- A recorded map is used, therefore it was difficult to navigade in a room with
a lot of unknown objects. That lead to a jumping of the estimated position of the
rc car, this also can appear if moving objects are introduced while the slalom is 
performed.

--------------------------------------------------------------------------------
## Outlook
--------------------------------------------------------------------------------
- A direct driving and steering could be implemented (twist messages) to follow
a smoother trajectory (Instead of goal publishing)
- A smarter clustering could be implemented, so far just a certain amout of points
are considered to be a cone (5-25). If e.g. width would be also condsidered, it 
would be even more robust
- Fine tuning of local and slalom parameters

--------------------------------------------------------------------------------
# Required Packages/Libaries:
--------------------------------------------------------------------------------
- teb_local_planner package
- pcl libaries 
- Eigen3
- actionlib
- move_base_msgs
- roscpp
- pcl_conversions
- pcl_ros
- laser_geometry
- dynamic_reconfigure

--------------------------------------------------------------------------------
# Launch slalom:
--------------------------------------------------------------------------------


1. -- SSH launches: --

`roslaunch tas launch_all_carlike_slalom.launch`


2. -- Rviz lokal --

`roslaunch tas rviz.launch`


3. -- Set initial position in rviz --


4. -- SSH launch slalom --

`roslaunch slalom slalom.launch`


5. -- (optinal) Local set mode static or dynamic in rqt_reconfigure in topic "slalom"

`rosrun rqt_reconfigure rqt_reconfigure`


6. -- Set initial position in rviz again --


7. -- Pull trigger to start automatic mode and drive the slalom --
