#!/usr/bin/python
import rospy
import cv2
import sys
import numpy as np
#import donkeycar as dk
from sensor_msgs.msg import Image
#from donkeycar.parts.camera import Webcam
#from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge, CvBridgeError
import time

def image_publisher():
    
    bridge = CvBridge()

    rospy.init_node('image_publisher', anonymous=True)

    pub = rospy.Publisher('/camera_image',Image, queue_size=100)

    print("Trying to open resource: /dev/video0")
    cap = cv2.VideoCapture(0)
    '''
    if not cap.isOpened():
        print("Error opening resource: " + str("/dev/video0"))
        print("Maybe opencv VideoCapture can't open it")
        exit(0)
    rate = rospy.Rate(10)

    print("Correctly opened resource, starting to show feed")
    rval, frame = cap.read()
    while rval:
        rval, frame = cap.read()

        # ROS image stuff
        if frame is not None:
            frame = np.uint8(frame)
            image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            pub.publish(image_message)
            cap.release()

        key = cv2.waitKey(20)
        rate.sleep()
    '''
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rval, cam_img = cap.read() 
        image = np.uint8(cam_img)
        image_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        #rospy.loginfo('converted image')

        pub.publish(image_msg)
        #rospy.loginfo('published image')
        rate.sleep()


    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        rospy.loginfo("shutting down publisher")
        ctrl_c = True

if __name__ == '__main__':
    
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
