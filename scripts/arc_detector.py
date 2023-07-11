#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool

class arc_detector():
    """ Class to detect the obstacles in red or yellow collors """

    def threshold_colors_hsv(self):
        """ Define the threshold for the frame in HSV """

        # saturation thresholds and value
        self.MINSAT = 145
        self.MAXSAT = 255
        self.MINVAL = 100
        self.MAXVAL = 255

        # thresholds red color
        self.MINRED = 140
        self.MAXRED = 220 

        # thresholds yellow color
        self.MINYELLOW = 0 
        self.MAXYELLOW = 15

    def imlimiares(self, hsv, hsvMin, hsvMax):
        """ Define the imlimiares for the arc """

        hsvtresh = cv2.inRange(hsv, hsvMin, hsvMax)
        hsvtresh = self.contour(hsvtresh)

        return hsvtresh
    
    def contour(self, img):
        """ Find the contours for shape analysis """
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hull = []
        for cnt in contours:
            hull.append(cv2.convexHull(cnt))
            self.area = cv2.contourArea(cnt)
            if self.area >= 2500:
                self.pub_collor_arc.publish(Bool(True))
                #print(self.area) for debug
            else:
                self.pub_collor_arc.publish(Bool(False))

        cv2.drawContours(img, hull, 0, 255, -1)
        return img

    def arc_colors(self, img):
        """ Transform the image into hsv and segment the arc colors """

        self.img = img

        # convert self.img for HSV
        self.img = cv2.GaussianBlur(img,(3,3), 0)
        hsv_frame = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        self.arc_red = self.imlimiares(hsv_frame, (self.MINRED, self.MINSAT, self.MINVAL), 
                                      (self.MAXRED, self.MAXSAT, self.MAXVAL))
        '''self.arc_yellow = self.imlimiares(hsv_frame, (self.MINYELLOW, self.MINSAT, self.MINVAL), 
                                         (self.MAXYELLOW, self.MAXSAT, self.MAXVAL))'''

        hsv_array = np.array(hsv_frame)
        h, s, v = np.split(hsv_array, 3, axis = 2)       
        
    def show(self):
        cv2.imshow('Acr color detected', self.arc_red)
        #cv2.imshow('Arc yellow detected', self.arc_yellow)

    def publishers(self):
        self.pub_collor_arc = rospy.Publisher("/arc_colors_avoidance_node/red", Bool, queue_size = 1)
        
    def __init__(self) -> None:
        self.threshold_colors_hsv()
        self.publishers()