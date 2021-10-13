#!/usr/bin/env python

import rospy

from  media_pipe_ros1_msg.msg import MediaPipeHumanHandList
import numpy as np
import matplotlib.pyplot as plt
import math	
from geometry_msgs.msg import Twist
from custom_msg.msg import set_angles,status_arm

global maoEsquerda, maoDireita, cmd_vel, msg;

msg = Twist()
msg_3R = set_angles()
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10);
cmd_3R = rospy.Publisher('/cmd_3R', set_angles, queue_size=10);

def getTopicoData():
	global maoEsquerda, maoDireita;
	maoEsquerda = []
	maoDireita = []
	
	msg = rospy.wait_for_message("/mediapipe/human_hand_list", MediaPipeHumanHandList)
	if (msg.human_hand_list):
		if (msg.human_hand_list.left_hand_key_points):
			for i in range(0,len(msg.human_hand_list.left_hand_key_points)):
				maoEsquerda.append(msg.human_hand_list.left_hand_key_points[i])
		
		if (msg.human_hand_list.right_hand_key_points):
			for i in range(0,len(msg.human_hand_list.right_hand_key_points)):
				maoDireita.append(msg.human_hand_list.right_hand_key_points[i])
	


def calculaDifAngular(dados):
	#Calcula a diferenca angular
	difAngular = math.atan2(dados[1][0]-dados[0][0], dados[1][1]-dados[0][1])
	difAngular = abs(math.degrees(difAngular))
	#difAngular = (difAngular +360)%360 # para ele ir de 0 a 360
	#print difAngular
	return (difAngular)

def calculaDifAngularSemAbs(dados):
	#Calcula a diferenca angular
	difAngular = math.atan2(dados[1][0]-dados[0][0], dados[1][1]-dados[0][1])
	difAngular = math.degrees(difAngular)
	#difAngular = (difAngular +360)%360 # para ele ir de 0 a 360
	#print difAngular
	return (difAngular)


def controleVelocidade():
	global maoEsquerda, maoDireita, msg,msg_3R, cmd_vel, cmd_3R;
	global linear 
	global angular 
        
	if (len(maoEsquerda)>0):
        	if (maoEsquerda[5]): 
			angle = calculaDifAngular([[maoEsquerda[5].x, maoEsquerda[5].y],[maoEsquerda[8].x, maoEsquerda[8].y]])
			
			if (angle>=100):
				linear = 3
			elif(angle < 100):
				linear = -3
		else:
        		linear = 0
	else:
        	linear = 0
			
	if (len(maoEsquerda)>0):
		if (maoEsquerda[0]): 
			angle = calculaDifAngularSemAbs([[maoEsquerda[0].x, maoEsquerda[0].y],[maoEsquerda[4].x, maoEsquerda[4].y]])
			
			if (angle>=0):
				print("Maior")
				angular = 10
			elif(angle < 0):
				print("Menor")
				angular = -10
		else:
        		angular = 0
	else:
        	angular = 0	


#__________________braco___________________
	set_OMB = 90
	set_COT = 90
	set_PUN = 90
	if (len(maoDireita)>0):
		if (maoDireita[5]): 
			set_OMB = calculaDifAngular([[maoDireita[5].x, maoDireita[5].y],[maoDireita[6].x, maoDireita[6].y]])
			set_COT = calculaDifAngular([[maoDireita[6].x, maoDireita[6].y],[maoDireita[7].x, maoDireita[7].y]])
			set_PUN = calculaDifAngular([[maoDireita[7].x, maoDireita[7].y],[maoDireita[8].x, maoDireita[8].y]])
		else:
			set_OMB = set_OMB
			set_COT = set_COT
			set_PUN = set_PUN
	else:
			set_OMB = 90
			set_COT = 90
			set_PUN =  90
			
	maoEsquerda = []
	maoDireita = []

	msg.linear.x = linear	
	msg.angular.z  = angular
	print(set_OMB)
	print(set_COT)
	print(set_PUN)
	msg_3R.set_OMB = -(90-set_OMB)
	msg_3R.set_COT = -(90-set_COT)
	msg_3R.set_PUN = -(90-set_PUN)
	cmd_vel.publish(msg)
	cmd_3R.publish(msg_3R)

		

# def paradeDeEmergencia():
# 	global maoEsquerda, maoDireita, cmd_vel, msg;
# 	if (len(maoEsquerda)>0):
# 		if (calculaDifAngular([[maoEsquerda[3].x, maoEsquerda[3].y],[maoEsquerda[4].x, maoEsquerda[4].y]])>= 90):
# 			if (calculaDifAngular([[maoEsquerda[10].x, maoEsquerda[10].y],[maoEsquerda[12].x, maoEsquerda[12].y]]) >= 90):
# 				if (calculaDifAngular([[maoEsquerda[14].x, maoEsquerda[14].y],[maoEsquerda[16].x, maoEsquerda[16].y]]) >= 100):
# 					if (calculaDifAngular([[maoEsquerda[18].x, maoEsquerda[18].y],[maoEsquerda[20].x, maoEsquerda[20].y]]) >= 100):
# 						msg.linear.x = 0	
# 						msg.angular.y = 0
# 						cmd_vel.publish(msg)
# 						print("parada de emergencia")
# 						return True;
	
# 	return False

def reset():
	global maoEsquerda, maoDireita, cmd_vel, msg;
	if (len(maoDireita)>0):
		if (calculaDifAngular([[maoDireita[3].x, maoDireita[3].y],[maoDireita[4].x, maoDireita[4].y]])>= 90):
			if (calculaDifAngular([[maoDireita[10].x, maoDireita[10].y],[maoDireita[12].x, maoDireita[12].y]]) >= 90):
				if (calculaDifAngular([[maoDireita[14].x, maoDireita[14].y],[maoDireita[16].x, maoDireita[16].y]]) >= 100):
					if (calculaDifAngular([[maoDireita[18].x, maoDireita[18].y],[maoDireita[20].x, maoDireita[20].y]]) >= 100):
						msg.set_OMB = -40	
						msg.set_COT = 145
						msg.set_PUN = -133
						cmd_vel.publish(msg)
						print("reset")
						return True;
	
	return False

def plotDados():
	global maoEsquerda, maoDireita;
	x = []
	y = []
	x2 = []
	y2 = []
	if (len(maoEsquerda)>0):
		print(maoEsquerda[0].x)
		for i in range(0,20):
			x.append(maoEsquerda[i].x)
			y.append(maoEsquerda[i].y)
			x2.append(maoEsquerda[i].x - maoEsquerda[0].x)
			y2.append(maoEsquerda[i].y - maoEsquerda[0].y)
		x = [x2[6],x2[8]]
		y = [y2[6],y2[8]]
		calculaDifAngular(x,y)
		#plt.scatter(x, y)
		#plt.show()

def init():
	while not rospy.is_shutdown():
		getTopicoData()
		controleVelocidade()
		# if (paradeDeEmergencia() == False):
		# 	controleVelocidade()
		# if (reset() == False):
		# 	reset()
		

if __name__=="__main__":
    rospy.init_node("comandosPorGestos")
    init();
    rospy.spin()

