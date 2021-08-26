#!/usr/bin/env python
import sys
import copy
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
from  media_pipe_ros1_msg.msg import MediaPipeHumanHolisticList
import numpy as np
import matplotlib.pyplot as plt
import math	
from geometry_msgs.msg import Twist

global poseHuman,leftHandHuman,rightHandHuman, msg, name,duration,client,cmd_vel, msg_vel;
msg_vel = Twist()
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10);
duration=0.0
name= "arm_with_torso_controller"


def getTopicoData():
    global poseHuman,leftHandHuman,rightHandHuman
    poseHuman = []
    leftHandHuman = []
    rightHandHuman = []

    msg = rospy.wait_for_message("/mediapipe/human_holistic_list", MediaPipeHumanHolisticList)
    if (msg.human_pose_list):
        for i in range(0,len(msg.human_pose_list)):
            poseHuman.append(msg.human_pose_list[i])

    if (msg.human_hand_list.left_hand_key_points):
        for i in range(0,len(msg.human_hand_list.left_hand_key_points)):
            leftHandHuman.append(msg.human_hand_list.left_hand_key_points[i])
    if (msg.human_hand_list.right_hand_key_points):
        for i in range(0,len(msg.human_hand_list.left_hand_key_points)):
            rightHandHuman.append(msg.human_hand_list.right_hand_key_points[i])



def calculaDifAngular(dados):
	#Calcula a diferenca angular
    
	difAngular = math.atan2( dados[1][1]-dados[0][1],dados[1][0]-dados[0][0])
	difAngular = math.degrees(difAngular)
	#print difAngular
	return (difAngular)

def controle():
    global poseHuman,duration,name,leftHandHuman,rightHandHuman;
    
    if (len(poseHuman)>0):
        
        #shoulder_pan_joint = calculaDifAngular([[poseHuman[11].x, poseHuman[11].z],[poseHuman[13].x, poseHuman[13].z]])
        shoulder_pan_joint = 0.0
        shoulder_lift_joint = calculaDifAngular([[leftHandHuman[6].x, leftHandHuman[6].y],[leftHandHuman[7].x, leftHandHuman[7].y]])
        elbow_flex_joint = calculaDifAngular([[leftHandHuman[7].x, leftHandHuman[7].y],[leftHandHuman[8].x, leftHandHuman[8].y]])
        wrist_flex_joint = calculaDifAngular([[leftHandHuman[8].x, leftHandHuman[8].y],[leftHandHuman[8].x, leftHandHuman[8].y]])
        #shoulder_pan_joint =  shoulder_pan_joint *0.01
        shoulder_lift_joint =  shoulder_lift_joint *0.01
        elbow_flex_joint = elbow_flex_joint*0.01
        wrist_flex_joint = wrist_flex_joint * 0.01
        #ombro_z = ombro_z *0.01
   
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                        "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint","torso_lift_joint"]
        pose = [0.0, shoulder_lift_joint, 0.0, elbow_flex_joint, 0.0, wrist_flex_joint, 0.0, 0.0]
        trajectory = JointTrajectory()
        trajectory.joint_names = joints
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = pose
        trajectory.points[0].velocities = [0.0 for _ in pose]
        trajectory.points[0].accelerations = [0.0 for _ in pose]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        return follow_goal

def controleVelocidade():
    global rightHandHuman, msg_vel, cmd_vel;
    global linear 
    global angular 
        
    if (len(rightHandHuman)>0):
        if (rightHandHuman[5]): 
            angle = abs(calculaDifAngular([[rightHandHuman[5].x, rightHandHuman[5].y],[rightHandHuman[8].x, rightHandHuman[8].y]]))
			
            if (angle>=95):
                linear = 1
               
            elif(angle < 85):
                linear = -1
               
        else:
            linear = 0
    else:
        linear = 0
    if (len(rightHandHuman)>0):
        if (rightHandHuman[4]): 
            angle = calculaDifAngular([[rightHandHuman[4].x, rightHandHuman[4].y],[rightHandHuman[8].x, rightHandHuman[8].y]])
            if (angle>=95):
                angular = 1
                print("maior")
            elif(angle < 85):
                angular = -1
                print("menor")
        else:
                angular = 0
    else:
            angular = 0		

    msg_vel.linear.x = linear	
    msg_vel.angular.y  = angular
    cmd_vel.publish(msg_vel)

def main(args=None):
    global poseHuman,duration,name;
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for %s..." % name)
    client.wait_for_server()

    while not rospy.is_shutdown():
        getTopicoData()
        controleVelocidade()
        follow_goal =controle()
        client.send_goal(follow_goal)
        client.wait_for_result() 
        


    
    rospy.spin()



if __name__ == '__main__':
    print("COCOZAO")
    main()