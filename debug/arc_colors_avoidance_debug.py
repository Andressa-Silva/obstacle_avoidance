#!/usr/bin/env python3
import cv2
import numpy as np

cap = cv2.VideoCapture('/home/andressa/catkin_ws/src/obstacle_avoidance/debug/videos/yellow.mp4')
#cap = cv2.VideoCapture('/home/andressa/catkin_ws/src/obstacle_avoidance/debug/videos/red.mp4')

video_cod = cv2.VideoWriter_fourcc(*'X264')
video_output = cv2.VideoWriter('captured_video.avi', video_cod, 1, (320,240))

# define yellow color range
MIN_YELLOW = np.array([0,60,80]) #([0,60,80]) 
MAX_YELLOW = np.array([30,255,255])

# define red color range
MIN_RED = np.array([160,100,20])
MAX_RED = np.array([179,255,255])
MIN_RED_L = np.array([0,100,20])
MAX_RED_L = np.array([10,255,255])

while(True):
    _, frame = cap.read()
                
    frame = cv2.GaussianBlur(frame, (3,3), 0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #mask = cv2.inRange(hsv_frame, MIN_YELLOW, MAX_YELLOW)
    mask   = cv2.inRange(hsv_frame, MIN_RED, MAX_RED)
    mask_L = cv2.inRange(hsv_frame, MIN_RED_L, MAX_RED_L)
    mask   = mask + mask_L 
    mask   = cv2.erode(mask, None, iterations = 2)
    mask   = cv2.dilate(mask, None, iterations = 6)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    hull = []
    for cnt in contours:
        hull.append(cv2.convexHull(cnt))
        area = cv2.contourArea(cnt)
        cv2.drawContours(mask, hull, 0, 255, -1)
        if area >= 3000:
            print("AAAAAAAAAAAA")

    mask = cv2.drawContours(mask, hull, 0, 255, -1)

    video_output.write(mask)  
    cv2.imshow('output',mask)

    if cv2.waitKey(1) & 0xFF == ord('Q'):
      break

cap.release()
video_output.release()
cv2.destroyAllWindows() 