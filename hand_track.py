import cv2
import sys
import mediapipe as mp
import os
import uuid
import numpy as np
import math
import pygame
import threading

#variables for camera
cam_source1 = "http://192.168.1.4:4747/video"
cam_source2=0
write_video=True
imagewrite=False

#variables for servo angle calculation

#servo1 (angle)
s1_max=170
s1_min=10
s1_mid=90

#cordinate boundary
d_max=200
d_min=0
var_max=300
var_min=0

#hyp max
hyp_max=200

#arm_length
arm_l=110





#initlizing 
hand_dect=mp.solutions.hands
draw=mp.solutions.drawing_utils
cam=cv2.VideoCapture(cam_source2)

# video writer
if write_video:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 60.0, (640, 480))


if not cam.isOpened():
    print("failed to open camera ")
    sys.exit()

def v_map(x,rmin,rmax,omin,omax):
    return (((x-rmin)*(omax-omin))/(rmax-rmin))+omin

#model
def detect(flipped_frame):
    #detection XD
    rgb_img=cv2.cvtColor(flipped_frame,cv2.COLOR_BGR2RGB)
    rgb_img.flags.writeable=False
    rslt=hands.process(rgb_img)   #model result
    return rslt

#draw the trace
def draw_hand(rslt,flipped_frame):
   
    for num,hand in enumerate(rslt.multi_hand_landmarks):
        draw.draw_landmarks(flipped_frame,hand,hand_dect.HAND_CONNECTIONS,
                            draw.DrawingSpec(color=(255, 0, 255),thickness=2,circle_radius=4),
                            draw.DrawingSpec(color=(255,176,18),thickness=2,circle_radius=4))
    #print(rslt.multi_hand_landmarks[0].landmark[0])
                    

#getting servo angles
def get_servo_angles(landmarks):
    servo_angles=[]

    #required landmarks
    #palm
    wrist=np.array([landmarks.landmark[0].x,landmarks.landmark[0].y])
    pp=np.array([landmarks.landmark[5].x,landmarks.landmark[5].y])
    ip=np.array([landmarks.landmark[17].x,landmarks.landmark[17].y])
    mp=np.array([landmarks.landmark[9].x,landmarks.landmark[9].y])
    
    
   

    #finger tips
    finger_tips=[]
    for i in range(4,21,8):
        finger_tips.append(np.array([landmarks.landmark[i].x,landmarks.landmark[i].y]))


    #get angle for base (wrist[0] middle_finger[1] angle) 
    angle=wrist-finger_tips[1]
    angle=math.atan2(angle[1],angle[0])
    angle=abs(round(math.degrees(angle)))
    angle=max(s1_min,min(s1_max,angle))
    servo_angles.append(angle)
   
    
    #adjust paramerters ____________________________________________________________ 
    # get x
    x=landmarks.landmark[9].y
    x=round(v_map((0.8-x),0.2,0.8,var_min,200))
    x=round(max(d_min,min(d_max,x)))
    servo_angles.append(x)

    #get z
    palmsize=np.linalg.norm(pp-wrist)+np.linalg.norm(ip-wrist)+np.linalg.norm(wrist-mp)
    z=v_map((0.8-palmsize),0.1,0.8,var_min,var_max)
    z=round(max(d_min,min(d_max,z)))
    servo_angles.append(z)


    #clamp
    dist1=np.linalg.norm(finger_tips[0]-finger_tips[2])
    dist2=np.linalg.norm(finger_tips[0]-finger_tips[1])
    if dist1 and dist2 <=0.1:
        servo_angles.append(True)
    else :
        servo_angles.append(False)

    return servo_angles

    






    

    

#pygame_____________________________________________________________  
def pygame_(a):
    pygame.init()
    screen=pygame.display.set_mode((400,400))
    clock=pygame.time.Clock()
    h=110





    
    while True:
 
        clock.tick(60)
        screen.fill((255,255,255))
        
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                pygame.quit()
                exit()
            if event.type==pygame.MOUSEMOTION:
                pos=np.array(event.pos)
        
        pygame.draw.line(screen,color="red",
                         start_pos=[200,150],
                         end_pos=[200-100*math.cos(math.radians(a.value[0])),
                                  150-100*math.sin(math.radians(a.value[0]))])
        
        x=a.value[1]
        y=a.value[2]

        hyp=math.sqrt(x**2+y**2)
        if hyp<=220:

            t1=math.atan2(y,x)
            q=math.acos((hyp/2)/h)

            a1=t1+q
            a2=t1-q

            a1x=200+h*math.cos(a1)
            a1y=400-h*math.sin(a1)

            a2x=a1x+h*math.cos(a2)
            a2y=a1y-h*math.sin(a2)


                    



            pygame.draw.line(screen,color="blue",width=10,start_pos=(200,400),end_pos=(a1x,a1y))
            pygame.draw.line(screen,color="red",width=10,start_pos=(a1x,a1y),end_pos=(a2x,a2y))
        

       
        
    
        pygame.display.update()




servo_angles = threading.Event()
servo_angles.value = [90,100,90,90,False]  
pygame_thread = threading.Thread(target=pygame_,args=(servo_angles,))
pygame_thread.start()
#__________________________________________________________________




if __name__=="__main__":
    
    with hand_dect.Hands(min_detection_confidence=0.8,min_tracking_confidence=0.4,max_num_hands=1) as hands:
        while True:
            #capture frame
            ret,frame=cam.read()
            if not ret:
                print("failed to capture frame")
                sys.exit()
 

            flipped_frame=cv2.flip(frame,1)

            rslt=detect(flipped_frame)

            
            if rslt.multi_hand_landmarks:
                draw_hand(rslt,flipped_frame)
                servo_angles.value=get_servo_angles(rslt.multi_hand_landmarks[0])
                cv2.putText(flipped_frame, str(servo_angles.value), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
            #showing image
            cv2.imshow("hand_detection",flipped_frame)

            #writing video/ image if needed
            if write_video:
                out.write(flipped_frame)
            if imagewrite:
                cv2.imwrite(os.path.join("image_captures","{}.jpg".format(uuid.uuid1())),flipped_frame)

            #exit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                if write_video:
                    out.release()
                break


    cam.release()
    cv2.destroyAllWindows()

