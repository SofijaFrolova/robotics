import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import os
import easygopigo3 as go
import time


myRobot = go.EasyGoPiGo3()
myRobot.set_speed(50)
#myRobot.spin_right()

lines = None
if os.path.isfile('./trackbar_defaults.txt'):
    with  open("trackbar_defaults.txt", 'r') as reader:
        lines = reader.readlines()
        lH = int(lines[0])
        lS = int(lines[1])
        lV = int(lines[2])
        uH = int(lines[3])
        uS = int(lines[4])
        uV = int(lines[5])
        #x = int(lines[6])
        #y = int(lines[7])

else:
    lH = 0
    lS = 97
    lV = 158
    uH = 171
    uS = 255
    uV = 255
    #x = 2
    #y = 2


def x_value(new_value):
    # Make sure to write the new value into the global variable
    pass
    
def y_value(new_value):
    # Make sure to write the new value into the global variable
    pass


def lH_find_value(new_value):
    # Make sure to write the new value into the global variable
    global lH
    lH = new_value
    
def lS_find_value(new_value):
    # Make sure to write the new value into the global variable
    global lS
    lS = new_value
    
def lV_find_value(new_value):
    # Make sure to write the new value into the global variable
    global lV
    lV = new_value
    
def uH_find_value(new_value):
    # Make sure to write the new value into the global variable
    global uH
    uH = new_value
    
def uS_find_value(new_value):
    # Make sure to write the new value into the global variable
    global uS
    uS = new_value
    
def uV_find_value(new_value):
    # Make sure to write the new value into the global variable
    global uV
    uV = new_value


cv2.namedWindow("Output")
#cv2.createTrackbar("Example trackbar", "Output", trackbar_value, 96, updateValue)
#cv2.createTrackbar("x", "Output", 1, 101, x_value)
#cv2.createTrackbar("y", "Output", 1, 101, y_value)


cv2.createTrackbar("lH", "Output", lH, 255, lH_find_value)
cv2.createTrackbar("lS", "Output", lS, 255, lS_find_value)
cv2.createTrackbar("lV", "Output", lV, 255, lV_find_value)
cv2.createTrackbar("uH", "Output", uH, 255, uH_find_value)
cv2.createTrackbar("uS", "Output", uS, 255, uS_find_value)
cv2.createTrackbar("uV", "Output", uV, 255, uV_find_value)

blobparams = cv2.SimpleBlobDetector_Params()

#blobparams.minThreshold = 0
#blobparams.maxThreshold = 255
blobparams.filterByArea = True
blobparams.minArea = 100
blobparams.maxArea = 100000

blobparams.filterByCircularity = False
blobparams.minCircularity = 0.8
blobparams.minDistBetweenBlobs = 100
blobparams.filterByInertia = False
blobparams.minInertiaRatio = 0.5
blobparams.filterByConvexity = False
blobparams.minConvexity = 0.9

detector = cv2.SimpleBlobDetector_create(blobparams)
cap = cv2.VideoCapture(0)

count = 1
previous = time.time()
state = 'FIRST'
pillars_in_middle = False

while True:
     
    start = time.time()
    time_duration = start - previous
    previous = start
    # Read the image from the camera
    ret, frame = cap.read()
    ret, frame = cap.read()
    ret, frame = cap.read()
    ret, frame = cap.read()
          
    frame = frame[350:400]
    
    #Returns trackbar position
    '''
    kernel_sizex = cv2.getTrackbarPos("x", "Output")
    kernel_sizex = 2*kernel_sizex+1
    kernel_sizey = cv2.getTrackbarPos("y", "Output")
    kernel_sizey = 2*kernel_sizey+1
    '''
    
    
    #blur = cv2.GaussianBlur(frame,(kernel_sizex,kernel_sizey),0)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([lH,lS,lV])
    upper_orange = np.array([uH,uS,uV])
    
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = cv2.bitwise_not(mask)
    height,width = mask.shape
    #print(width)
    
    #print(width)
    thresholded = cv2.rectangle(mask, (0, 0), (width-1, height-1), (255, 255, 255), 2)
    # Write some text onto the frame
   
   
    keypoints = detector.detect(thresholded) #objects thresholded
    # sort by x
    keypoints = sorted(keypoints, key=lambda kp : kp.pt[0])
    #print(keypoints)
    
    for keypoint in keypoints:
            keypoint = (int(keypoint.pt[0]), int(keypoint.pt[1]),)
            cv2.putText(frame, str(keypoint), keypoint, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    if len(keypoints) >= 2:
        #myRobot.stop()
        left = keypoints[0].pt[0]
        right = keypoints[1].pt[0]
        #print("Vasak", keypoints[0].pt[0])
        #print("Parem", keypoints[1].pt[0])
        pillar_center = int((left+right)/2)
        cv2.line(frame, (pillar_center, 0),(pillar_center, height), (255,255,255), 2)
        
    
    centre_upper = width/2 + 20
    centre_lower = width/2 - 20
    low = width/2 - 65
    up = width/2 + 65
    
    if state == 'FIRST':
        myRobot.set_speed(30)
        myRobot.spin_right()
        
        if len(keypoints) >= 1:
            state = 'SECOND'
    
    elif state == 'SECOND':
        myRobot.set_speed(20)
        
        if len(keypoints) >= 2:
            #myRobot.stop()
            
            if pillar_center < centre_lower:
                myRobot.spin_left()
                print('spin_right')
            elif  pillar_center > centre_upper:
                myRobot.spin_right()
                print('spin_left')
            else:
                if pillars_in_middle:
                    state = 'FORWARD'
                else:
                    state = 'FIND_CENTRE'
        
    elif state == 'FIND_CENTRE':
        myRobot.set_speed(20)
        if len(keypoints) ==  1:
            state = 'FIRST'
        elif len(keypoints) >= 2:
            if abs(keypoints[0].size - keypoints[1].size) < 20:
                state = 'SECOND'
                pillars_in_middle = True
            elif keypoints[0].size > keypoints[1].size:
                myRobot.turn_degrees(90)
                myRobot.drive_cm(10)
                myRobot.turn_degrees(-93)
                myRobot.drive_cm(5)
                
            elif keypoints[0].size < keypoints[1].size:
                myRobot.turn_degrees(-90)
                myRobot.drive_cm(10)
                myRobot.turn_degrees(93)
                myRobot.drive_cm(5)
    
    elif state == 'CENTRE':
        myRobot.set_speed(60)
        if pillar_center <= centre_lower:
            myRobot.spin_left()
                #print('spin_right')
        elif  pillar_center > centre_upper:
            myRobot.spin_right()
                #print('spin_left')
        else:
            state = 'SECOND'
            

    elif state == 'FORWARD':
        myRobot.set_speed(120)
        myRobot.drive_cm(10)
        
        if len(keypoints) >= 2:
            if abs(keypoints[0].pt[0] - keypoints[1].pt[0]) >= 300:
                state = 'FINAL'
            state = "SECOND"
        elif len(keypoints) == 0:
            state == 'FINAL'
        else:
            state = 'FINAL'
            
    elif state == 'FINAL':
        myRobot.set_speed(120)
        myRobot.forward()
    

        
    print(state, len(keypoints))

    
    if time_duration != 0:
        cv2.putText(frame, str(count/time_duration), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
    
    cv2.imshow('Processed', thresholded)    
    cv2.imshow('blur', frame)
        
    # Quit the program when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        lines[0] = str(lH) + '\n'
        lines[1] = str(lS) + '\n'       
        lines[2] = str(lV) + '\n'
        lines[3] = str(uH) + '\n'
        lines[4] = str(uS) + '\n'
        lines[5] = str(uV) + '\n'
        #lines[6] = str(x) + '\n'
        #lines[7] = str(y) + '\n'
        break

   
#Save trackbar values to the file
if os.path.isfile('./trackbar_defaults.txt'):
    with open('./trackbar_defaults.txt', 'w') as writer:
        writer.writelines(lines)

       

myRobot.stop()
# When everything done, release the capture
print('closing program')
cap.release()
cv2.destroyAllWindows()
