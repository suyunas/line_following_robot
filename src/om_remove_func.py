#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

###### Functions ########
def open_gripper(gripper_group,gripper_group_variable_values):
	print ('Opening Gripper...')
	gripper_group_variable_values[0] = 00.019
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(5)

def close_gripper(gripper_group,gripper_group_variable_values):
	print ('Closing Gripper...')
	gripper_group_variable_values[0] = 0.006
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(5)

def move_home(arm_group):
	arm_group.set_named_target("home")
	print ('Executing Move: Home')
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(5)

def move_position1(arm_group):
	arm_group.set_named_target("position1")
	print ('Executing Move: Position1')
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(5)

def move_position2(arm_group):
	arm_group.set_named_target("position2")
	print ('Executing Move: Position2')
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(5)

def move_position3(arm_group):
	arm_group.set_named_target("position3")
	print ('Executing Move: Position3')
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(5)

def main():
	names1 = 'position1'
	values1 = [0.0,1.1,-0.9,0.75]
	names2 = 'position2'
	values2 = [1.5,-1.0,0.3,0.7]
	names3 = 'position3'
	values3 = [1.5,0.7,0.0,0.7]
	###### Setup ########
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	arm_group = moveit_commander.MoveGroupCommander("arm")
	gripper_group = moveit_commander.MoveGroupCommander("gripper")
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

	#Had probelms with planner failing, Using this planner now. I believe default is OMPL
	arm_group.set_planner_id("RRTConnectkConfigDefault")
	#Increased available planning time from 5 to 10 seconds
	arm_group.set_planning_time(10);

	arm_group.remember_joint_values(names1, values1)
	arm_group.remember_joint_values(names2, values2)
	arm_group.remember_joint_values(names3, values3)

	gripper_group_variable_values = gripper_group.get_current_joint_values()

	rate = rospy.Rate(5)
	ctrl_c = False
	###### Main ########
	#rate.sleep()
	move_home(arm_group)
	#rate.sleep()
	open_gripper(gripper_group,gripper_group_variable_values)
	#rate.sleep()
	move_position1(arm_group)
	#rate.sleep()
	close_gripper(gripper_group, gripper_group_variable_values)
	#rate.sleep()
	move_home(arm_group)
	#rate.sleep()
	move_position2(arm_group)
	#rate.sleep()
	move_position3(arm_group)
	#rate.sleep()
	open_gripper(gripper_group,gripper_group_variable_values)
	#rate.sleep()
	move_position2(arm_group)
	#rate.sleep()
	close_gripper(gripper_group,gripper_group_variable_values)
	#rate.sleep()
	move_home(arm_group)
	#rate.sleep()

	#moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()