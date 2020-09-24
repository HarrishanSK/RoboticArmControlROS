#!/usr/bin/env python
#Student Name: Harrishan Sureshkumar
#Student ID: 150294245
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from std_msgs.msg import Float32 
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

s = 0.0 #Dynamic value for the length of the side of the square


class MoveGroupPythonInterface(object):

  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Here a RobotCommander object is instantiated. Is the outer level interface to robot
    robot = moveit_commander.RobotCommander()

    #Here a PlanningSceneInterface object is instantiated and is an interface
    scene = moveit_commander.PlanningSceneInterface()

    #Name of robot
    group_name = "panda_arm"
    #Here a MoveGroupCommander object is instantiated. Is an interface to one group of joints - in this case 7 joints for pandra_arm
    group = moveit_commander.MoveGroupCommander(group_name)

    #Here a DisplayTrajectory publisher is created to be later used to publish trajectories to be visualized in RViz
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


    #Gets name of reference frame for this robot
    planning_frame = group.get_planning_frame()
   # print "============ Reference frame: %s" % planning_frame

    #Gets end-effector link for this group
    eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    #Gets a list of all the groups in the robot
    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Use when debugging to print entire state of robot
    #print "======================"
    #print "Printing robot state"
    #print robot.get_current_state()
    #print "======================"


    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  #Planning to a joint goal, used to make robot go to starting configuration
  def go_to_joint_state(self):
    group = self.group
    
    #Here we can get joint values from the group and adjust some values
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    #Move robot to starting configuration
    group.go(joint_goal, wait=True)

    #Call stop() to ensure no residual movement
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  #Plan a cartesian path by directly specifying a list of waypoints for the end-effector. When executing interactively in python scale must be set to 1.0
  def plan_cartesian_path(self, scale=1.0):

    group = self.group
    global s #Use obtained length of size of square from node 1

    waypoints = []# waypoints to follow

    wpose = group.get_current_pose().pose

    wpose.position.x += scale * s  # 1st move - in x
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += scale * s  # 2nd move + in y
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= scale * s  # 3rd move + in x
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * s  # 4th move - in y
    waypoints.append(copy.deepcopy(wpose))

    #Make 0.01 as the eef_step in Cartesian translation so cartesian path can be interpolated at a resolution os 1cm. Disable jump threshold by making it 0.0.
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold


    # move_group wont actually move yet this is only making a plan
    return plan, fraction


  #Display trajectory of plan. 
  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    #The msg DisplayTrajectory has 2 primary fields which are called trajectory and trajectory_start. The trajectory_start field is populated with our current robot state to add our plan to the trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish diplay trajectory
    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):
    group = self.group

    #To follow the plan that has been computed, run .execute
    group.execute(plan, wait=True)

    #If the robots current joint state is not within some tolerance of the 1st waypoint in RobotTrajectory or execute(), it will fail.


#Method used to test if list of values happen to be within a tolerance of their counterparts in another list
def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

#This function obtains the duration of the panda arm to act out its animation to traverse its planned trajectory in R-viz
def get_animation_duration(cartesian_plan, delay):
    	    #Calculate time it takes to draw animation
		#Time from start to last point in trajectory.. [-1] represents last point in array
	    time_s = float(cartesian_plan.joint_trajectory.points[-1].time_from_start.secs)#in secs
	    time_ns = float(cartesian_plan.joint_trajectory.points[-1].time_from_start.nsecs)/ (10**9)#and remaining nano secs converted to secs

	    duration = time_s + time_ns #add them together to get duration in secs
	    animation_duration = duration * delay #introduce delay to counter lag of virtual machine/limited resources
            return animation_duration

#This function draws a square using the panda arm in rviz
def draw_square(size):

    global s #size of length of sides of square
    s = size.data
    global panda
    delay = 1.85 #delay to counter the lag caused by Virtual Machine/Limited resources

    print "------------------------------------------------------------------"
    print "------------------------------------------------------------------"
    print("Move Panda - Received square size, s = " + str(round(s,4))) #Print message in terminal
#Rounded to 4 decimal places as shown in QMplus example video
    try:

	    #Go to starting configuration
	    print "------------------------------------------------------------------"
	    print "------------------------------------------------------------------"
	    print "Move Panda - Going to start configuration"
	    panda.go_to_joint_state()
	    rospy.sleep(2) #pause execution for 2secs whilst arm moves to starting point


	    #Plan motion trajectory
	    print "------------------------------------------------------------------"
	    print "------------------------------------------------------------------"
	    print "Move Panda - Planning motion trajectory"
	    cartesian_plan, fraction = panda.plan_cartesian_path()#get cartesian plan
            animation_duration = get_animation_duration(cartesian_plan,delay)#get animation duration
 	    rospy.sleep(animation_duration)#pause execution whilst animation plays out in R-viz


	    #Show planned trajectory
	    print "------------------------------------------------------------------"
	    print "------------------------------------------------------------------"
	    print "Move Panda - Showing planned trajectory (Animation duration is " + str(round(animation_duration,2)) + " seconds)"
	    panda.display_trajectory(cartesian_plan)
	    rospy.sleep(animation_duration)


	    #Execute planned trajectory
	    print "------------------------------------------------------------------"
	    print "------------------------------------------------------------------"
	    print "Move Panda - Executing planned trajectory"
	    panda.execute_plan(cartesian_plan)  
	    print "------------------------------------------------------------------"
	    print "------------------------------------------------------------------"
    	    print("Move Panda - Waiting for desired size of square trajectory") #Print message in terminal

    except:
        print "Draw Square Error"


panda = MoveGroupPythonInterface() #Global vaiable to move panda arm robot (Object of class)

def main():
  try:
    rospy.init_node('move_panda_square', anonymous=True)#initialise node
    print("Node2: Move Panda Square Activated") #Print message in terminal
    print "------------------------------------------------------------------"
    print "------------------------------------------------------------------"
    print("Move Panda - Waiting for desired size of square trajectory") #Print message in terminal
    rospy.Subscriber('squareSizeGen', Float32, draw_square)#subscribes to topic published by node1 (squareSizeGen) & calls draw_square() func with obtaibed random value 's' for length of side of new square

    rospy.spin()#keeps python from exiting until this node is stopped
    print "Move Panda Complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
