import rospy
import cv2
import mediapipe as mp
from media_pipe_ros1_msg.msg import HandPoint,HandPoint,MediaPipeHumanHand,MediaPipeHumanHolisticList                            
from mediapipe.python.solutions.pose import PoseLandmark
from mediapipe.framework.formats import landmark_pb2
import numpy as np
from sensor_msgs.msg import Image                            
from cv_bridge import CvBridge, CvBridgeError

import io
from PIL import Image as pi

cap = cv2.VideoCapture(0)
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose
mp_face_mesh = mp.solutions.face_mesh
global face_connections
face_connections = []
bridge = CvBridge()

# def callback(data):
#         global Imagem;
#         bridge = CvBridge()
#         im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
   
#         Imagem = im


NAME_POSE = [
    (PoseLandmark.NOSE), (PoseLandmark.LEFT_EYE_INNER),
    (PoseLandmark.LEFT_EYE), (PoseLandmark.LEFT_EYE_OUTER),
    (PoseLandmark.RIGHT_EYE_INNER), ( PoseLandmark.RIGHT_EYE),
    (PoseLandmark.RIGHT_EYE_OUTER), ( PoseLandmark.LEFT_EAR),
    (PoseLandmark.RIGHT_EAR), ( PoseLandmark.MOUTH_LEFT),
    (PoseLandmark.MOUTH_RIGHT), ( PoseLandmark.LEFT_SHOULDER),
    (PoseLandmark.RIGHT_SHOULDER), ( PoseLandmark.LEFT_ELBOW),
    (PoseLandmark.RIGHT_ELBOW), ( PoseLandmark.LEFT_WRIST),
    (PoseLandmark.RIGHT_WRIST), ( PoseLandmark.LEFT_PINKY),
    (PoseLandmark.RIGHT_PINKY), ( PoseLandmark.LEFT_INDEX),
    (PoseLandmark.RIGHT_INDEX), ( PoseLandmark.LEFT_THUMB),
    (PoseLandmark.RIGHT_THUMB), ( PoseLandmark.LEFT_HIP),
    (PoseLandmark.RIGHT_HIP), ( PoseLandmark.LEFT_KNEE),
    (PoseLandmark.RIGHT_KNEE), ( PoseLandmark.LEFT_ANKLE),
    (PoseLandmark.RIGHT_ANKLE), ( PoseLandmark.LEFT_HEEL),
    (PoseLandmark.RIGHT_HEEL), ( PoseLandmark.LEFT_FOOT_INDEX),
    (PoseLandmark.RIGHT_FOOT_INDEX)
]


class HolisticPublisher():
    
    def __init__(self):
           
        rospy.init_node('mediapipe_publisher_holistic')
        self.publisher_ = rospy.Publisher( '/mediapipe/human_holistic_list',MediaPipeHumanHolisticList,  queue_size=10)
        # rospy.Subscriber("usb_cam/image_raw", Image, callback)

    def getimage_callback(self):
        global face_connections, Imagem
        mediapipehumanholisticlist = MediaPipeHumanHolisticList() 
        mediapipehuman = MediaPipeHumanHand()
        points = HandPoint()
        drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
      
        with mp_face_mesh.FaceMesh(
                max_num_faces=1,
               
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as face_mesh,mp_hands.Hands(
                static_image_mode=False,                 
                min_detection_confidence=0.7,
                min_tracking_confidence=0.7,
                max_num_hands=2) as hands, mp_pose.Pose(
               min_detection_confidence=0.5,
               min_tracking_confidence=0.5) as pose:
            
            while not rospy.is_shutdown():
                
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                # If loading a video, use 'break' instead of 'continue'.
                    continue
                # if Imagem != "":
                #msg = rospy.wait_for_message("usb_cam/image_raw", Image)
                #bridge = CvBridge()
                
                #orig = bridge.imgmsg_to_cv2(msg, "bgr8")
               # print(orig)
                #im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                #im = np.array(pi.open(io.BytesIO(msg.data))) 
                image =image
             
                image = image                                
                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)                
                image.flags.writeable = False
                results_face_mesh = face_mesh.process(image)
                results_pose = pose.process(image)
                results_hands= hands.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                imageHeight, imageWidth, _ = image.shape
                landmark_temp = []
                #face process
                if results_face_mesh.multi_face_landmarks:
                    index_face_mesh = 0
                        
                    while index_face_mesh <= 467:                       

                        landmark_temp.append(results_face_mesh.multi_face_landmarks[0].landmark[index_face_mesh]) 
                        mediapipehumanholisticlist.human_face_mesh_list[index_face_mesh].x = results_face_mesh.multi_face_landmarks[0].landmark[index_face_mesh].x
                        mediapipehumanholisticlist.human_face_mesh_list[index_face_mesh].y = results_face_mesh.multi_face_landmarks[0].landmark[index_face_mesh].y
                        mediapipehumanholisticlist.human_face_mesh_list[index_face_mesh].z = results_face_mesh.multi_face_landmarks[0].landmark[index_face_mesh].z
                        index_face_mesh = index_face_mesh +1
                            
                    landmark_subset = landmark_pb2.NormalizedLandmarkList(landmark = landmark_temp)
                    mp_drawing.draw_landmarks(
                            image=image,
                            landmark_list=landmark_subset,
                            connections=None,
                            landmark_drawing_spec=drawing_spec,
                            connection_drawing_spec=drawing_spec)
                else: # responsavel por mandar 0 nos topicos quando corpo nao esta na tela
                    index_pose = 0
                    for point in mp_pose.PoseLandmark:                          
                                                                                            
                        mediapipehumanholisticlist.human_pose_list[index_pose].name = str(NAME_POSE[index_pose])
                        mediapipehumanholisticlist.human_pose_list[index_pose].x = 0.0
                        mediapipehumanholisticlist.human_pose_list[index_pose].y = 0.0
                        mediapipehumanholisticlist.human_pose_list[index_pose].z = 0.0
                        mediapipehumanholisticlist.human_pose_list[index_pose].visibility = 0.0
                        index_pose = index_pose +1

                    

                #     #processo pose
                #     # Draw the pose annotation on the image.
                # # image.flags.writeable = True
                #     #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                # mp_drawing.draw_landmarks(
                #     image, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                # if results_pose.pose_landmarks != None:
                #     index_pose = 0
                #     for pose_landmarks in (results_pose.pose_landmarks.landmark):
                            
                #         mediapipehumanholisticlist.human_pose_list[index_pose].name = str(NAME_POSE[index_pose])
                #         mediapipehumanholisticlist.human_pose_list[index_pose].x = pose_landmarks.x
                #         mediapipehumanholisticlist.human_pose_list[index_pose].y = pose_landmarks.y
                #         mediapipehumanholisticlist.human_pose_list[index_pose].z = pose_landmarks.z
                #         mediapipehumanholisticlist.human_pose_list[index_pose].visibility = pose_landmarks.visibility
                #         index_pose = index_pose +1

                #     mediapipehumanholisticlist.num_humans = 1
                #     self.publisher_.publish(mediapipehumanholisticlist)
                # else: # responsavel por mandar 0 nos topicos quando corpo nao esta na tela
                #     index_pose = 0
                #     for point in mp_pose.PoseLandmark:                          
                                                                                        
                #         mediapipehumanholisticlist.human_pose_list[index_pose].name = str(NAME_POSE[index_pose])
                #         mediapipehumanholisticlist.human_pose_list[index_pose].x = 0.0
                #         mediapipehumanholisticlist.human_pose_list[index_pose].y = 0.0
                #         mediapipehumanholisticlist.human_pose_list[index_pose].z = 0.0
                #         mediapipehumanholisticlist.human_pose_list[index_pose].visibility = 0.0
                #         index_pose = index_pose +1

                    #HAND PROCESS        
                    
                if results_hands.multi_hand_landmarks != None:
                    
                    hand_number_screen = 0 # index de controle de quantas maos aparecem na tela                  
                        #esse for passa pela quantidades de m??o na tela setada como maximo 2 no momento
                    for hand_landmarks, handedness in zip(results_hands.multi_hand_landmarks,results_hands.multi_handedness):
                        # x = [landmark.x for landmark in hand_landmarks.landmark]
                        # y = [landmark.y for landmark in hand_landmarks.landmark] 
                        # center = np.array([np.mean(x)*imageWidth, np.mean(y)*imageHeight]).astype('int32')
                        # cv2.rectangle(image, (center[0]-110,center[1]-110), (center[0]+110,center[1]+110), (255,0,0), 1)                        
                        if handedness.classification[0].label == "Right":
                            mp_drawing.draw_landmarks(
                            image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                            index_point = 0
                                
                            for point in mp_hands.HandLandmark:  
                                                             
                                
                                normalizedLandmark = hand_landmarks.landmark[point]                                                                
                                mediapipehuman.right_hand_key_points[index_point].name = str(point) 
                                mediapipehuman.right_hand_key_points[index_point].x = normalizedLandmark.x
                                mediapipehuman.right_hand_key_points[index_point].y = normalizedLandmark.y
                                mediapipehuman.right_hand_key_points[index_point].z = normalizedLandmark.z                                
                                if hand_number_screen == 0:
                                    mediapipehuman.left_hand_key_points[index_point].name = str(point) 
                                    mediapipehuman.left_hand_key_points[index_point].x = 0.0
                                    mediapipehuman.left_hand_key_points[index_point].y = 0.0
                                    mediapipehuman.left_hand_key_points[index_point].z = 0.0
                                index_point = index_point +1
                            hand_number_screen = hand_number_screen +1

                        elif handedness.classification[0].label == "Left":
                            mp_drawing.draw_landmarks(
                            image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                            index_point = 0
                                
                            for point in mp_hands.HandLandmark:
                                    
                                normalizedLandmark = hand_landmarks.landmark[point]  
                                points.name = str(point)                            
                                mediapipehuman.left_hand_key_points[index_point].name = str(point) 
                                mediapipehuman.left_hand_key_points[index_point].x = normalizedLandmark.x
                                mediapipehuman.left_hand_key_points[index_point].y = normalizedLandmark.y 
                                mediapipehuman.left_hand_key_points[index_point].z = normalizedLandmark.z
                                    
                                if hand_number_screen == 0:
                                    mediapipehuman.right_hand_key_points[index_point].name = str(point) 
                                    mediapipehuman.right_hand_key_points[index_point].x = 0.0
                                    mediapipehuman.right_hand_key_points[index_point].y = 0.0
                                    mediapipehuman.right_hand_key_points[index_point].z = 0.0
                                index_point = index_point +1
                            hand_number_screen = hand_number_screen +1

                        
                else: # responsavel por mandar 0 nos topicos quando as duas maos nao estao na tela
                    index_point = 0
                    for point in mp_hands.HandLandmark:                          
                                                                                            
                        mediapipehuman.right_hand_key_points[index_point].name = str(point) 
                        mediapipehuman.right_hand_key_points[index_point].x = 0.0
                        mediapipehuman.right_hand_key_points[index_point].y = 0.0
                        mediapipehuman.right_hand_key_points[index_point].z = 0.0 
                        mediapipehuman.left_hand_key_points[index_point].name = str(point) 
                        mediapipehuman.left_hand_key_points[index_point].x = 0.0
                        mediapipehuman.left_hand_key_points[index_point].y = 0.0
                        mediapipehuman.left_hand_key_points[index_point].z = 0.0
                        index_point = index_point + 1 

                    


                mediapipehumanholisticlist.human_hand_list.right_hand_key_points = mediapipehuman.right_hand_key_points
                mediapipehumanholisticlist.human_hand_list.left_hand_key_points = mediapipehuman.left_hand_key_points
                mediapipehumanholisticlist.num_humans = 1
                mediapipehumanholisticlist.image_raw = bridge.cv2_to_imgmsg(image, encoding="passthrough")
                self.publisher_.publish(mediapipehumanholisticlist)

                    
                # cv2.imshow('MediaPipe holistic', image)
                # if cv2.waitKey(5) & 0xFF == 27:
                #     break        

def main(args=None):
    # global face_connections
    for x in mp_face_mesh.FACE_CONNECTIONS:
        print(x)
    #     face_connections.append(x[0])
    #     face_connections.append(x[1])
 
    holistic_publisher = HolisticPublisher()
    holistic_publisher.getimage_callback()
     
    
    cap.release()
    
    rospy.spin()

if __name__ == '__main__':
    main()
