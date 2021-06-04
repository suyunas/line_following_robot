#!/usr/bin/env python 

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveKobuki
#import om_remove_func
import sys
import copy
import moveit_commander
import moveit_msgs.msg

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image_raw",Image,self.camera_callback)
        self.movekobuki_object = MoveKobuki()
        self.tot_count = 0
        self.control_on = 1
        self.control_off = 0
        self.counter2 = 0
        #rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        #lower_green = np.array([55,100,100])
        #upper_green = np.array([85,255,255])
        lower_magenta= np.array([130,100,100])
        upper_magenta= np.array([160,255,255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask2 = cv2.inRange(hsv, lower_magenta, upper_magenta)
        zero_counter = np.count_nonzero(mask2==0)
        rospy.loginfo(zero_counter)
        if zero_counter > 999 and zero_counter< 12000:
            self.tot_count = self.tot_count + 1
            rospy.loginfo(self.tot_count)
        if self.tot_count > 1 and self.control_on == 1:
        #    self.movekobuki_object.stop_robot()
        #    raw_input('Enter desired movement: ') 
            self.movekobuki_object.stop_robot()
            names1 = 'position1'
            values1 = [0.04,1.2,-0.9,0.5]
            names2 = 'position2'
            values2 = [1.5,-1.0,0.3,0.7]
            names3 = 'position3'
            values3 = [1.5,0.7,0.0,0.7]

            ###### Functions ########
            def open_gripper():
                print ('Opening Gripper...')
                gripper_group_variable_values[0] = 00.019
                gripper_group.set_joint_value_target(gripper_group_variable_values)
                plan2 = gripper_group.go()
                gripper_group.stop()
                gripper_group.clear_pose_targets()
                rospy.sleep(5)

            def close_gripper():
                print ('Closing Gripper...')
                gripper_group_variable_values[0] = 0.006
                gripper_group.set_joint_value_target(gripper_group_variable_values)
                plan2 = gripper_group.go()
                gripper_group.stop()
                gripper_group.clear_pose_targets()
                rospy.sleep(5)

            def move_home():
                arm_group.set_named_target("home")
                print ('Executing Move: Home')
                plan1 = arm_group.plan()
                arm_group.execute(plan1, wait=True)
                arm_group.stop()
                arm_group.clear_pose_targets()
                variable = arm_group.get_current_pose()
                print (variable.pose)
                rospy.sleep(5)

            def move_position1():
                arm_group.set_named_target("position1")
                print ('Executing Move: Position1')
                plan1 = arm_group.plan()
                arm_group.execute(plan1, wait=True)
                arm_group.stop()
                arm_group.clear_pose_targets()
                variable = arm_group.get_current_pose()
                print (variable.pose)
                rospy.sleep(5)

            def move_position2():
                arm_group.set_named_target("position2")
                print ('Executing Move: Position2')
                plan1 = arm_group.plan()
                arm_group.execute(plan1, wait=True)
                arm_group.stop()
                arm_group.clear_pose_targets()
                variable = arm_group.get_current_pose()
                print (variable.pose)
                rospy.sleep(5)

            def move_position3():
                arm_group.set_named_target("position3")
                print ('Executing Move: Position3')
                plan1 = arm_group.plan()
                arm_group.execute(plan1, wait=True)
                arm_group.stop()
                arm_group.clear_pose_targets()
                variable = arm_group.get_current_pose()
                print (variable.pose)
                rospy.sleep(5)

            ###### Setup ########
            moveit_commander.roscpp_initialize(sys.argv)
            #rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

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

            ###### Main ########
            move_home()
            open_gripper()
            move_position1()
            close_gripper()
            move_home()
            move_position2()
            move_position3()
            open_gripper()
            move_position2()
            close_gripper()
            move_home()

            #moveit_commander.roscpp_shutdown() 
            self.tot_count = 0
            self.control_on = self.control_off
            #self.counter2 = self.counter2 + 1

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)
        
        
        error_x = cx - width / 2;
        twist_object = Twist();
        twist_object.linear.x = 0.2 * 0.10;
        twist_object.angular.z = (-error_x / 200) * 0.10;
        #rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        # Make it start turning
        self.movekobuki_object.move_robot(twist_object)
        
    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    line_follower_object = LineFollower()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
        exit()
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()