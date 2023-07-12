#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool

class arc_detector():
    """ Class to detect the obstacles in red or yellow collors """
   
    def threshold_colors_hsv(self):
        """ Define the threshold for the arcs collor in HSV """
        
        # define yellow color range
        self.MIN_YELLOW = np.array([0,60,80]) 
        self.MAX_YELLOW = np.array([30,255,255])

        # define red color range
        self.MIN_RED     = np.array([160,100,20])
        self.MAX_RED     = np.array([179,255,255])
        self.MIN_RED_LOW = np.array([0,100,20])
        self.MAX_RED_LOW = np.array([10,255,255])

    def imlimiares_yellow(self, hsv, hsvMin, hsvMax):
        """ Define the imlimiares for the arc """

        hsvtresh = cv2.inRange(hsv, hsvMin, hsvMax)
        hsvtresh = self.contour(hsvtresh)

        return hsvtresh
    
    def imlimiares_red(self, hsv, hsvMin, hsvMax, hsvMin_low, hsvMax_low):
        """ Define the imlimiares for the arc """

        hsvtresh  = cv2.inRange(hsv, hsvMin, hsvMax)
        hsvtresh_ = cv2.inRange(hsv, hsvMin_low, hsvMax_low)
        hsvtresh  = hsvtresh + hsvtresh_
        
        hsvtresh = self.contour(hsvtresh)
        
        return hsvtresh
    
    def contour(self, mask):
        """ Find the contours for shape analysis """

        mask = cv2.erode(mask, None, iterations = 2)
        mask = cv2.dilate(mask, None, iterations = 6)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hull = []
        for cnt in contours:
            hull.append(cv2.convexHull(cnt))
            self.area = cv2.contourArea(cnt)
            cv2.drawContours(mask, hull, 0, 255, -1)
            if self.area >= 2500:
                self.pub_collor_arc.publish(Bool(True))
            else:
                self.pub_collor_arc.publish(Bool(False))

        mask = cv2.drawContours(mask, hull, 0, 255, -1)
        return mask

    def arc_colors(self, img):
        """ Transform the image into hsv and segment the arc colors """
        self.img = img

        # convert self.img for HSV
        self.img = cv2.GaussianBlur(img, (3,3), 0)
        hsv_frame = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        self.arc_red = self.imlimiares_red(hsv_frame, self.MIN_RED, self.MAX_RED, self.MIN_RED_LOW, self.MAX_RED_LOW)
        self.arc_yellow = self.imlimiares_yellow(hsv_frame, self.MIN_YELLOW, self.MAX_YELLOW)   
        
    def show(self):
        """ Imshow for help in debug """

        cv2.imshow('Acr red detected', self.arc_red)
        cv2.imshow('Arc yellow detected', self.arc_yellow)
        
        cv2.waitKey(100)

    def publishers(self):
        self.pub_collor_arc = rospy.Publisher("/arc_colors_avoidance_node/red", Bool, queue_size = 1)
        
    def __init__(self) -> None:
        self.publishers()
        self.threshold_colors_hsv()