AR_Week8_test8 readme.txt

The tasks 

Create a ROS package that will automatically generate Cartesian space movements of the end-effector pf the Panda robot manipulator

the first part consisted in creating all essential ROS packages, which were succesfully created 

PART1 -- THE PACKAGE: rospy,moveit_commander, moveit_msgs, std_msgs

PART2 -- THE key environments were succesfully installed in the catkin workspace

PART3 -- THE Two ROS nodes:

# 1) responsible for generating the random square size: square_size_generator.py
# 2) responsible for the panda arm movement : move_panda_square.py
have been successfully implemented.

PART4 -- PLOT:All the positions of the arms of the Panda arm have been plotted on rqt_plt

PART5 -- RECORDING 

A final video of the simulation was recorded by using KAZAM:
 
However in the video  there are two planned trajectories operating before the execution of the planned trajectory; one is coming from RVIZ and the other one is the one implemented in the code, however when i switched off the Robot visual from RVIZ the trajectory was not outputted any more, so i preferred having 2 un-synchronized display planned trajectories rather than nothing.


 
