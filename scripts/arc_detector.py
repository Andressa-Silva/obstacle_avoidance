#!/usr/bin/env python3
import rospy
import numpy as np
import cv2

class arc_detector():
    """ Class to detect the obstacles in red or yellow"""

    def threshold_colors_hsv(self):
        """ Define the threshold for the frame in HSV"""

        # saturation thresholds and value
        self.MINSAT = 30
        self.MAXSAT = 255
        self.MINVAL = 50
        self.MAXVAL = 255

        # thresholds red color
        RED = 170
        DRED = 10
        self.MINRED = RED - DRED
        self.MAXRED = RED + DRED

        # thresholds yellow color
        YELLOW = 25
        DYELLOW = 20
        self.MINYELLOW = YELLOW - DYELLOW
        self.MAXYELLOW = YELLOW + DYELLOW

    def imlimiares(self, hsv, hsvMin, hsvMax):
        """ Define the imlimiares for the arc"""

        # thresholds
        hmin, smin, vmin = hsvMin
        hmax, smax, vmax = hsvMax

        if hmin < hmax:
            hsvtresh = cv2.inRange(hsv, hsvMin, hsvMax)
        else:
            hsvtresh1 = cv2.inRange(hsv, (0, smin, vmin), (hmax, smax, vmax))
            hsvtresh2 = cv2.inRange(hsv, (hmin, smin, vmin), (180, smax, vmax))
            hsvtresh = cv2.bitwise_or(hsvtresh1, hsvtresh2, mask = None)

        hsvtresh = self.contour(hsvtresh)

        return hsvtresh
    
    def contour(self, img):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hull = []
        for cnt in contours:
            hull.append(cv2.convexHull(cnt))
            self.area = cv2.contourArea(cnt)

        cv2.drawContours(img, hull, 0, 255, -1)
        return img
    

    def arc_colors(self, img):
        self.img = img

        # convert self.img for HSV
        self.img = cv2.GaussianBlur(img,(3,3), 0)
        hsv_frame = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        self.arc_red = self.imlimiares(hsv_frame, (self.MINRED, self.MINSAT, self.MINVAL), 
                                      (self.MAXRED, self.MAXSAT, self.MAXVAL))
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        #self.arc_red = cv2.morphologyEx(self.arc_red, cv2.MORPH_CLOSE, kernel, iterations = 2)
        R1, T1, success1 = self.getRefFrame(self.arc_red, "_RED")

        self.arc_yellow = self.imlimiares(hsv_frame, (self.MINYELLOW, self.MINSAT, self.MINVAL), 
                                         (self.MAXYELLOW, self.MAXSAT, self.MAXVAL))
        #R2, T2, success2 = self.getRefFrame(self.arc_yellow, "_YELLOW")

        hsv_array = np.array(hsv_frame)
        h, s, v = np.split(hsv_array, 3, axis = 2)
        print(h)

        #return R1, T1, success1, R2, T2, success2
        
        
    def show(self):
        cv2.imshow('Acr color detected', self.arc_red)
        #cv2.imshow('Arc yellow detected', self.arc_yellow)
        
    def __init__(self) -> None:
        self.threshold_colors_hsv()