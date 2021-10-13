#!/usr/bin/env python
import rospy
from Tkinter import *
from subprocess import Popen
from media_pipe_ros1_msg.msg import HandPoint,HandPoint,MediaPipeHumanHand,MediaPipeHumanHolisticList 
import numpy as np
import math # 'math' needed for 'sqrt'
import keyboard  # using module keyboard
import time
global leftHandHuman, rightHandHuman
global pontoReferenciaSeg1, pontoReferenciaSeg2, pontoReferenciaSeg3, pontoReferenciaSeg4, pontoReferenciaSeg5
global ArrayAllPontos, ArrayPontosSeg1, ArrayPontosSeg2, ArrayPontosSeg3, ArrayPontosSeg4,ArrayPontosSeg5
global msg
global pararGravar
global start_time

leftHandHuman = []
rightHandHuman = []
pararGravar = True
pontoReferenciaSeg1 = 0
pontoReferenciaSeg2 = 0
pontoReferenciaSeg3 = 9
pontoReferenciaSeg4 = 13
pontoReferenciaSeg5 = 17  
ArrayAllPontos = []
ArrayPontosSeg1 = []
ArrayPontosSeg2 = []
ArrayPontosSeg3 = []
ArrayPontosSeg4 = []
ArrayPontosSeg5 = []
msg = MediaPipeHumanHolisticList()


# Freq = 2500
# Dur = 150

# top = Tkinter.Tk()
# top.title('Teste')
# top.geometry('200x100') # Size 200, 200

def start():
    global leftHandHuman
    global rightHandHuman
    global  pararGravar
    global ArrayAllPontos, ArrayPontosSeg1, ArrayPontosSeg2, ArrayPontosSeg3, ArrayPontosSeg4,ArrayPontosSeg5
    global start_time
    ArrayAllPontos = []
    ArrayPontosSeg1 = []
    ArrayPontosSeg2 = []
    ArrayPontosSeg3 = []
    ArrayPontosSeg4 = []
    ArrayPontosSeg5 = []
    leftHandHuman = []
    rightHandHuman = []
    print("O Teste comeca em:")
    for i in range(1,6):
        time.sleep(1)
        print('%s segundo' % (6-i))
      
        if i == 5:
            start_time = time.time()
            print("Gravando...")
            pararGravar = False
    


def stop():
    global  pararGravar,ArrayAllPontos, ArrayPontosSeg1
    pararGravar = True
    SomaDPSeg1 = 0
    SomaDPSeg2 = 0
    SomaDPSeg3 = 0
    SomaDPSeg4 = 0
    SomaDPSeg5 = 0
    #print(len(ArrayAllPontos))
    print("________Calculando Seguimentos...______________")
    
    #____________Seguimento 1___________________________
    print ("_________Seguimento 1_____________")
    #cria variaveis globais
    for x in range (0, len(ArrayPontosSeg1[0][0])):
            globals()['arraySomaSeg1%s' % x] = []
    #atribui a
    for i in range(0,len(ArrayPontosSeg1)): 
        for x in range (0, len(ArrayPontosSeg1[0][0])):
            globals()['arraySomaSeg1%s' % x].append(ArrayPontosSeg1[i][0][x])    
    #printa as variaveis:
    for x in range (0, len(ArrayPontosSeg1[0][0])):
        SomaDPSeg1 = SomaDPSeg1 + np.std(globals()['arraySomaSeg1%s' % x], ddof = 1)
        print(np.std(globals()['arraySomaSeg1%s' % x], ddof = 1))
    #___________________________________________________
        #____________Seguimento 2___________________________
    print ("_________Seguimento 2_____________")
    #cria variaveis globais
    for x in range (0, len(ArrayPontosSeg2[0][0])):
            globals()['arraySomaSeg2%s' % x] = []
    #atribui a
    for i in range(0,len(ArrayPontosSeg2)): 
        for x in range (0, len(ArrayPontosSeg2[0][0])):
            globals()['arraySomaSeg2%s' % x].append(ArrayPontosSeg2[i][0][x])    
    #printa as variaveis:
    for x in range (0, len(ArrayPontosSeg2[0][0])):
        SomaDPSeg2 = SomaDPSeg2 + np.std(globals()['arraySomaSeg2%s' % x], ddof = 1)
        print(np.std(globals()['arraySomaSeg2%s' % x], ddof = 1))
    # #___________________________________________________
    #     #____________Seguimento 3___________________________
    # print ("_________Seguimento 3_____________")
    # #cria variaveis globais
    # for x in range (0, len(ArrayPontosSeg3[0][0])):
    #         globals()['arraySomaSeg3%s' % x] = []
    # #atribui a
    # for i in range(0,len(ArrayPontosSeg3)): 
    #     for x in range (0, len(ArrayPontosSeg3[0][0])):
    #         globals()['arraySomaSeg3%s' % x].append(ArrayPontosSeg3[i][0][x])    
    # #printa as variaveis:
    # for x in range (0, len(ArrayPontosSeg3[0][0])):
    #     SomaDPSeg3 = SomaDPSeg3 + np.std(globals()['arraySomaSeg3%s' % x], ddof = 1)
    #     print(np.std(globals()['arraySomaSeg3%s' % x], ddof = 1))
    # #___________________________________________________
    #     #____________Seguimento 4___________________________
    # print ("_________Seguimento 4_____________")
    # #cria variaveis globais
    # for x in range (0, len(ArrayPontosSeg4[0][0])):
    #         globals()['arraySomaSeg4%s' % x] = []
    # #atribui a
    # for i in range(0,len(ArrayPontosSeg4)): 
    #     for x in range (0, len(ArrayPontosSeg4[0][0])):
    #         globals()['arraySomaSeg4%s' % x].append(ArrayPontosSeg4[i][0][x])    
    # #printa as variaveis:
    # for x in range (0, len(ArrayPontosSeg4[0][0])):
    #     SomaDPSeg4 = SomaDPSeg4 + np.std(globals()['arraySomaSeg4%s' % x], ddof = 1)
    #     print(np.std(globals()['arraySomaSeg4%s' % x], ddof = 1))
    # #___________________________________________________
    #     #____________Seguimento 5___________________________
    # print ("_________Seguimento 5_____________")
    # #cria variaveis globais
    # for x in range (0, len(ArrayPontosSeg5[0][0])):
    #         globals()['arraySomaSeg5%s' % x] = []
    # #atribui a
    # for i in range(0,len(ArrayPontosSeg5)): 
    #     for x in range (0, len(ArrayPontosSeg5[0][0])):
    #         globals()['arraySomaSeg5%s' % x].append(ArrayPontosSeg5[i][0][x])    
    # #printa as variaveis:
    # for x in range (0, len(ArrayPontosSeg5[0][0])):
    #     SomaDPSeg5 = SomaDPSeg5 + np.std(globals()['arraySomaSeg5%s' % x], ddof = 1)
    #     print(np.std(globals()['arraySomaSeg5%s' % x], ddof = 1))
    # #___________________________________________________
    somaDPSegs = []
    somaDPSegs.append(SomaDPSeg1)
    somaDPSegs.append(SomaDPSeg2)
    # somaDPSegs.append(SomaDPSeg3)
    # somaDPSegs.append(SomaDPSeg4)
    # somaDPSegs.append(SomaDPSeg5)
    print ("_________Soma dos Seguimentos_____________")
    print('Seguimento 1 -- > %s' % SomaDPSeg1)
    print('Seguimento 2 -- > %s' % SomaDPSeg2)
    # print('Seguimento 3 -- > %s' % SomaDPSeg3)
    # print('Seguimento 4 -- > %s' % SomaDPSeg4)
    # print('Seguimento 5 -- > %s' % SomaDPSeg5)
    print ("______________________")
    maior = 0
    for i in range(0,len(somaDPSegs)):        
        if somaDPSegs[i] > maior:
            maior = somaDPSegs[i]
    
    for i in range(0,len(somaDPSegs)):
        if somaDPSegs[i] == maior: 
            print(somaDPSegs[i])  
            print(maior)         
            if i == 0:
                print('Mao Esquerda se moveu mais -- > %s' % somaDPSegs[i])
            if i == 1:
                print('Mao Direita se moveu mais -- > %s' % somaDPSegs[i])
                #CONTINUAR AQUI A LOGICA PARA SABER QUAL O DEDO 
                # for i in range(0,len(ArrayPontosSeg2)): 
                #     for x in range (0, len(ArrayPontosSeg2[0][0])):
                #         print('------------%s--------------' % x)
                #         print( globals()['arraySomaSeg2%s' % x])
                         
            # if i == 2:
            #     print('Seguimento 3 se moveu mais -- > %s' % somaDPSegs[i])
            # if i == 3:
            #     print('Seguimento 4 se moveu mais -- > %s' % somaDPSegs[i])
            # if i == 4:
            #     print('Seguimento 5 se moveu mais -- > %s' % somaDPSegs[i])

    
   

def getData():
    global leftHandHuman, rightHandHuman
    leftHandHuman = []
    rightHandHuman = []
    
    msg = rospy.wait_for_message("/mediapipe/human_holistic_list", MediaPipeHumanHolisticList)
    if (msg.human_hand_list):			
        if (msg.human_hand_list.left_hand_key_points):
            for i in range(0,len(msg.human_hand_list.left_hand_key_points)):
                leftHandHuman.append(msg.human_hand_list.left_hand_key_points[i])
        if (msg.human_hand_list.right_hand_key_points):
            for i in range(0,len(msg.human_hand_list.right_hand_key_points)):
                rightHandHuman.append(msg.human_hand_list.right_hand_key_points[i])

def calculaDifAngularDaReferencia(dados):
	#Calcula a diferenca angular
	difAngular = math.atan2(dados[1][0]-dados[0][0], dados[1][1]-dados[0][1])
	difAngular = (math.degrees(difAngular))
	difAngular = (difAngular +360)%360 # para ele ir de 0 a 360
	return (difAngular)




def GravarMovimento():
    global leftHandHuman
    global pontoReferenciaSeg1,pontoReferenciaSeg2,pontoReferenciaSeg3,pontoReferenciaSeg4,pontoReferenciaSeg5
    global ArrayPontos, pararGravar
    
    distanciasSegmento1 = []
    distanciasSegmento2 = []
    # distanciasSegmento3 = []
    # distanciasSegmento4 = []
    # distanciasSegmento5 = []

    referenciaSeg1 = [leftHandHuman[pontoReferenciaSeg1].x, leftHandHuman[pontoReferenciaSeg1].y]
    referenciaSeg2 = [rightHandHuman[pontoReferenciaSeg2].x, rightHandHuman[pontoReferenciaSeg2].y]
    # referenciaSeg3 = [leftHandHuman[pontoReferenciaSeg3].x, leftHandHuman[pontoReferenciaSeg3].y]
    # referenciaSeg4 = [leftHandHuman[pontoReferenciaSeg4].x, leftHandHuman[pontoReferenciaSeg4].y]
    # referenciaSeg5 = [leftHandHuman[pontoReferenciaSeg5].x, leftHandHuman[pontoReferenciaSeg5].y]
  
    #segmento 1
    distSeg1Pt0to1 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[1].x, leftHandHuman[1].y]])
    distSeg1Pt0to2 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[2].x, leftHandHuman[2].y]])
    distSeg1Pt0to3 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[3].x, leftHandHuman[3].y]])
    distSeg1Pt0to4 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[4].x, leftHandHuman[4].y]])
    
    distSeg1Pt0to5 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[5].x, leftHandHuman[5].y]])
    distSeg1Pt0to6 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[6].x, leftHandHuman[6].y]])
    distSeg1Pt0to7 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[7].x, leftHandHuman[7].y]])
    distSeg1Pt0to8 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[8].x, leftHandHuman[8].y]])

    distSeg1Pt0to9 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[9].x, leftHandHuman[9].y]])
    distSeg1Pt0to10 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[10].x, leftHandHuman[10].y]])
    distSeg1Pt0to11 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[11].x, leftHandHuman[11].y]])
    distSeg1Pt0to12 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[12].x, leftHandHuman[12].y]])

    distSeg1Pt0to13 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[13].x, leftHandHuman[13].y]])
    distSeg1Pt0to14 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[14].x, leftHandHuman[14].y]])
    distSeg1Pt0to15 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[15].x, leftHandHuman[15].y]])
    distSeg1Pt0to16 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[16].x, leftHandHuman[16].y]])

    distSeg1Pt0to17 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[17].x, leftHandHuman[17].y]])
    distSeg1Pt0to18 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[18].x, leftHandHuman[18].y]])
    distSeg1Pt0to19 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[19].x, leftHandHuman[19].y]])
    distSeg1Pt0to20 = calculaDifAngularDaReferencia([referenciaSeg1,
    [leftHandHuman[20].x, leftHandHuman[20].y]])

    #segmento 2
    distSeg2Pt0to1 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[1].x, rightHandHuman[1].y]])
    distSeg2Pt0to2 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[2].x, rightHandHuman[2].y]])
    distSeg2Pt0to3 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[3].x, rightHandHuman[3].y]])
    distSeg2Pt0to4 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[4].x, rightHandHuman[4].y]])
  
    distSeg2Pt0to5 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[5].x, rightHandHuman[5].y]])
    distSeg2Pt0to6 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[6].x, rightHandHuman[6].y]])
    distSeg2Pt0to7 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[7].x, rightHandHuman[7].y]])
    distSeg2Pt0to8 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[8].x, rightHandHuman[8].y]])

    distSeg2Pt0to9 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[9].x, rightHandHuman[9].y]])
    distSeg2Pt0to10 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[10].x, rightHandHuman[10].y]])
    distSeg2Pt0to11 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[11].x, rightHandHuman[11].y]])
    distSeg2Pt0to12 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[12].x, rightHandHuman[12].y]])

    distSeg2Pt0to13 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[13].x, rightHandHuman[13].y]])
    distSeg2Pt0to14 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[14].x, rightHandHuman[14].y]])
    distSeg2Pt0to15 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[15].x, rightHandHuman[15].y]])
    distSeg2Pt0to16 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[16].x, rightHandHuman[16].y]])

    distSeg2Pt0to17 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[17].x, rightHandHuman[17].y]])
    distSeg2Pt0to18 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[18].x, rightHandHuman[18].y]])
    distSeg2Pt0to19 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[19].x, rightHandHuman[19].y]])
    distSeg2Pt0to20 = calculaDifAngularDaReferencia([referenciaSeg2,
    [rightHandHuman[20].x, rightHandHuman[20].y]])


    distanciasSegmento1.append([
        distSeg1Pt0to1,
        distSeg1Pt0to2,
        distSeg1Pt0to3,
        distSeg1Pt0to4,
        distSeg1Pt0to5,
        distSeg1Pt0to6,
        distSeg1Pt0to7,
        distSeg1Pt0to8,
        distSeg1Pt0to9,
        distSeg1Pt0to10,
        distSeg1Pt0to11,
        distSeg1Pt0to12,
        distSeg1Pt0to13,
        distSeg1Pt0to14,
        distSeg1Pt0to15,
        distSeg1Pt0to16,
        distSeg1Pt0to17,
        distSeg1Pt0to18,
        distSeg1Pt0to19,
        distSeg1Pt0to20
        ])
    distanciasSegmento2.append([
        distSeg2Pt0to1,
        distSeg2Pt0to2,
        distSeg2Pt0to3,
        distSeg2Pt0to4,
        distSeg2Pt0to5,
        distSeg2Pt0to6,
        distSeg2Pt0to7,
        distSeg2Pt0to8,
        distSeg2Pt0to9,
        distSeg2Pt0to10,
        distSeg2Pt0to11,
        distSeg2Pt0to12,
        distSeg2Pt0to13,
        distSeg2Pt0to14,
        distSeg2Pt0to15,
        distSeg2Pt0to16,
        distSeg2Pt0to17,
        distSeg2Pt0to18,
        distSeg2Pt0to19,
        distSeg2Pt0to20
        ])

    ArrayPontosSeg1.append(distanciasSegmento1)
    ArrayPontosSeg2.append(distanciasSegmento2)
    
   

    
    
  



    
    

def init():
    global pararGravar
    global start_time
    print("Execute seu movimento em ate 3 segundos")
    while not rospy.is_shutdown():
        ROOT.update()
        getData()
        
        if pararGravar == False:
            current_time = time.time()
            elapsed_time = current_time - start_time
            GravarMovimento()
            
            if elapsed_time > 3:
                print("Gravacao Completa")
                stop()
          
           
            
            
            


ROOT = Tk()
LABEL = Label(ROOT, text="App Detector!")
startButton = Button(ROOT, height=2, width=20, text ="Iniciar", 
command = start)
stopButton = Button(ROOT, height=2, width=20, text ="Parar", 
command = stop)
LABEL.pack()
startButton.pack()
stopButton.pack()

if __name__ == '__main__':
    rospy.init_node('mediapipe_exp', anonymous=True)
    # startButton = tk.Button(msg_frame, height=2, width=20, text ="Start", 
    # command = start)
    # stopButton = tk.Button(msg_frame, height=2, width=20, text ="Stop", 
    # command = stop)

    # stopButton.pack()
    # startButton.pack()
    init()
    rospy.spin()
    
    