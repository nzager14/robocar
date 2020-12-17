#!/usr/bin/python
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time

# class BaseCamera:

#     def run_threaded(self):
#         return self.frame

class Image_Publisher():

    def __init__(self):
        rospy.init_node('image_publisher_node', anonymous=True)

        self.on = False

        self.pub = rospy.Publisher('/camera_image',Image,queue_size=100)

    # def image_pub(self):
    #     from datetime import datetime, timedelta
    #     import pygame.image
    #     while self.on:
    #         start = datetime.now()

    #         if self.cam.query_image():
    #             # snapshot = self.cam.get_image()
    #             # self.frame = list(pygame.image.tostring(snapshot, "RGB", False))
    #             snapshot = self.cam.get_image()
    #             snapshot1 = pygame.transform.scale(snapshot, self.resolution)
    #             self.frame = pygame.surfarray.pixels3d(pygame.transform.rotate(pygame.transform.flip(snapshot1, True, False), 90))
    #             self.pub.publish(frame)

    #         stop = datetime.now()
    #         s = 1 / self.framerate - (stop - start).total_seconds()
    #         if s > 0:
    #             time.sleep(s)

    #     self.cam.stop()

    # # def run_threaded(self):
    # #     return self.frame

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stoping Webcam')
        time.sleep(.5)    

    def main(self):

        vc = cv2.VideoCapture(0)
        
        if vc.isOpened(): 
            rval, frame = vc.read()
        else:
            print("Error opening resource: " + str('/dev/video0'))
            rval = False

        while rval:
            rval, frame = vc.read()

            self.pub.publish(frame)

            key = cv2.waitKey(20)
            
            if key == 27:  # exit on ESC
                shutdown()
                break 



        # rospy.init_node('image_publisher', anonymous=True)

        # pub = rospy.Publisher('camera_image', Image, queue_size=100)
        
        # print "Trying to open resource: /dev/video0 "
        # cap = cv2.VideoCapture("/dev/video0")
        # if not cap.isOpened():
        #     print "Error opening resource: " + str(resource)
        #     print "Maybe opencv VideoCapture can't open it"
        #     exit(0)


        # print "Correctly opened resource, starting to show feed."
        # rval, frame = cap.read()
        # while rval:
            
        #     rval, frame = cap.read()

        #     # ROS image stuff
        #     if frame is not None:
        #         frame = np.uint8(frame)

        #     pub.publish(image_message)

        #     key = cv2.waitKey(1)
            
        # cv2.destroyAllWindows()

    
if __name__ == '__main__':
    pub_obj = Image_Publisher()

    try:
        pub_obj.main()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    
