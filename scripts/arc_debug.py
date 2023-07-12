#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
import cv2
from arc_detector import arc_detector

def img_arc_callback():
    cap = cv2.VideoCapture("/home/andressa/catkin_ws/src/obstacle_avoidance/debug/videos/red.mp4")
    #cap = cv2.VideoCapture('/home/andressa/catkin_ws/src/obstacle_avoidance/debug/videos/yellow.mp4') 

    if cap.isOpened():
        while not rospy.is_shutdown(): 
            ret, frame = cap.read()

            arc = arc_detector()
            arc.arc_colors(frame)
            arc.show()                       

    else:
        print("Unable to open camera")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__': 
    rospy.init_node("arc_colors_avoidance_node")
    
    img_arc_callback()
    rospy.spin()
