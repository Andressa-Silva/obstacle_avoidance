#!/usr/bin/env python3
import rospy
import cv2
from arc_detector import arc_detector

def img_arc_callback():
    cap = cv2.VideoCapture(0)

    if cap.isOpened():
        while not rospy.is_shutdown(): 
            ret, frame = cap.read()

            print("Camera is Opened")
            arc = arc_detector()
            arc.arc_colors(frame)
            arc.show()

            #cv2.imshow('Cam opened', frame)
            cv2.waitKey(100)                       

    else:
        print("Unable to open camera")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__': 
    print("inicializing node")
    rospy.init_node("arc_colors_avoidance_node", anonymous = False)
    
    img_arc_callback()
    rospy.spin()