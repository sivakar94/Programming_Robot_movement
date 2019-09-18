# Name: Sivakar Sivarajah
#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from AR_week8_test.msg import length_params


############################################  Initiating_MOVE_PANDA_ARM_CLASS  ###################################################
# WE DIFINES THE OBJECT AND ALL THE METHOD WITHIN THE CLASS 

class MovePandaArm(object):

  def __init__(self,length): # included the length in the init 
    super(MovePandaArm, self).__init__()
    self.length=length
    
    moveit_commander.roscpp_initialize(sys.argv)  #initializing move_it commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()

    group_names = robot.get_group_names()
  
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
        print "----------------GOING TO STARTING CONFIGURATION------------------"
    	move_group = self.move_group

    	joint_goal = move_group.get_current_joint_values()
    	joint_goal[0] = 0
    	joint_goal[1] = -pi/4
    	joint_goal[2] = 0
    	joint_goal[3] = -pi/2
    	joint_goal[4] = 0
    	joint_goal[5] = pi/3
    	joint_goal[6] = 0

    	move_group.go(joint_goal, wait=True)
    	# Calling ``stop()`` ensures that there is no residual movement
    	move_group.stop()

  def plan_cartesian_path(self,length):
    	print "----------------PLANNING A CARTHESIAN PATH-----------------------"
    
    	move_group = self.move_group

    	global scale
    	scale = 1
    	waypoints = []
    	wpose = move_group.get_current_pose().pose
  
    	wpose.position.x += scale * length  #  MOVE FORWARD/ +x
    	waypoints.append(copy.deepcopy(wpose))
	
    	wpose.position.y += scale * length  # TURN LEFT/ +y
    	waypoints.append(copy.deepcopy(wpose))
    
    	wpose.position.x -= scale * length  # MOVE DOWNWARDS / -x
    	waypoints.append(copy.deepcopy(wpose))

    	wpose.position.y -= scale * length  # TURN RIGHT / -y
    	waypoints.append(copy.deepcopy(wpose))


    	(plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    	# Note: We are just planning, not asking move_group to actually move the robot yet:
    	return plan, fraction

  def display_trajectory(self, plan): # IT DISPLAY  THE ON Rviz BEFORE THE EXECUTION 
	print "----------------DISPLAYING PLANNED TRAJECTORY--------------------"
    	robot = self.robot
    	display_trajectory_publisher = self.display_trajectory_publisher
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    	display_trajectory.trajectory_start = robot.get_current_state()
    	display_trajectory.trajectory.append(plan)
    	display_trajectory_publisher.publish(display_trajectory)
	
    

  def execute_plan(self, plan): # IT EXECUTES THE MOVEMENT
        print "----------------EXECUTING PLANNED TRAJECTORY---------------------"    	
	move_group = self.move_group
	move_group.execute(plan, wait=True)

################################################################################### END OF THE CLASS##########################################


def wait_for_next():# method for waiting for the next message
        print "----------------WAITING FOR THE NEXT MESSAGE---------------------"    	
	


def callback(req):
        global l0
	l0 =req.l0  
	move = MovePandaArm(l0)	
        print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
        print "=================== Square size recieved is: %s================" % l0
        print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"

	#after the message is recieved the folllowing action are being opearated by the panda arm
        move.go_to_joint_state()
	rospy.sleep(2.0)
	cartesian_plan, fraction = move.plan_cartesian_path(l0)
	rospy.sleep(2.0)
	move.display_trajectory(cartesian_plan)
	rospy.sleep(2.0)
        move.execute_plan(cartesian_plan)
	wait_for_next()


#Node 2 Subscriber

def move_panda_square():   
	rospy.init_node('reader', anonymous=True) #subscribe to the first topic
        rospy.Subscriber('length', length_params,callback)
   
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=1)  
	move_panda_square()
	


