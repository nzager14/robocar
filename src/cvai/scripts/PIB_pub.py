#!/usr/bin/python

import os
import time
import rospy
import numpy as np
import cv2
from simple_pid import PID
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32


class LineFollower:
    '''
    OpenCV based controller
    This controller takes a horizontal slice of the image at a set Y coordinate.
    Then it converts to HSV and does a color thresh hold to find the yellow pixels.
    It does a histogram to find the pixel of maximum yellow. Then is uses that iPxel
    to guid a PID controller which seeks to maintain the max yellow at the same point
    in the image.
    '''
    def __init__(self):
        self.vert_scan_y = 60   # num pixels from the top to start horiz scan
        self.vert_scan_height = 10 # num pixels high to grab from horiz scan
        self.color_thr_low = np.asarray((0, 0, 200)) # hsv dark yellow
        self.color_thr_hi = np.asarray((255, 255, 255)) # hsv light yellow
        self.target_pixel = None # of the N slots above, which is the ideal relationship target
        self.steering = 0.0 # from -1 to 1
        self.throttle = 0.15 # from -1 to 1
        self.recording = True # Set to true if desired to save camera frames
        self.delta_th = 0.1 # how much to change throttle when off
        self.throttle_max = 0.3
        self.throttle_min = 0.15
        self.pid_st = PID(Kp=-0.01, Ki=0.00, Kd=-0.001,output_limits=(-1,1))
        #rospy.Subscriber("/camera_image",Image, self.image_handler)
        self.st_pub = rospy.Publisher('/steering_value', Int32, queue_size=1)
        self.th_pub = rospy.Publisher('/throttle_value', Int32, queue_size=1)
        self.bridge_object = CvBridge()
        self.cv_img = cv2.VideoCapture(0)
    
    def get_i_color(self, cam_img):
        '''
        get the horizontal index of the color at the given slice of the image
        input: cam_image, an RGB numpy array
        output: index of max color, value of cumulative color at that index, and mask of pixels in range 
        '''
        
        # take a horizontal slice of the image
        iSlice = self.vert_scan_y
        rospy.loginfo(cam_img)
        ''' 
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(cam_img, desired_encoding='rgb8')
            #cv_image = self.cam_sub
        except CvBridgeError as e:
            print(e)
        '''
        cv_image = np.uint8(cam_img)

        scan_line = cv_image[iSlice : iSlice + self.vert_scan_height, :, :]

        # convert to HSV color space
        img_hsv = cv2.cvtColor(scan_line, cv2.COLOR_RGB2HSV)

        # make a mask of the colors in our range we are looking for
        mask = cv2.inRange(img_hsv, self.color_thr_low, self.color_thr_hi)

        # which index of the range has the highest amount of yellow?
        hist = np.sum(mask, axis=0)
        max_yellow = np.argmax(hist)

        return max_yellow, hist[max_yellow], mask

    def run_PID(self):
        '''
        main runloop of the CV controller
        input: cam_image, an RGB numpy array
        output: steering, throttle, and recording flag
        '''
        rospy.loginfo('main node initiated')
        #rospy.spin()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            
            rval, frame = self.cv_img.read()
            rospy.loginfo(rval)
            rospy.loginfo(frame)
            max_yellow, confidense, mask = self.get_i_color(frame)
            self.cv_img.release()
            conf_thresh = 0.001
            
            if self.target_pixel is None:
                # Use the first run of get_i_color to set our relationship with the yellow line.
                # You could optionally init the target_pixel with the desired value.
                self.target_pixel = max_yellow

                # this is the target of our steering PID controller
                self.pid_st.setpoint = self.target_pixel

            elif confidense > conf_thresh:
                # invoke the controller with the current yellow line position
                # get the new steering value as it chases the ideal
                self.steering = self.pid_st(max_yellow)
            
                # slow down linearly when away from ideal, and speed up when close
                if abs(max_yellow - self.target_pixel) > 10:
                    if self.throttle > self.throttle_min:
                        self.throttle -= self.delta_th
                else:
                    if self.throttle < self.throttle_max:
                        self.throttle += self.delta_th

            self.st_pub.publish(self.steering)
            self.th_pub.publish(self.throttle)

            rate.sleep()



################################################
# import rospy
# from std_msgs.msg import Int32

# def PID_Publisher():
#     self.st_pub = rospy.Publisher('/steering_value', Int32, queue_size=1)
#     self.th_pub = rospy.Publisher('/throttle_value', Int32, queue_size=1)

#     rospy.init_node('PID_publisher_node', anonymous=True)
#     rate = rospy.Rate(10) # 10hz

#     while not rospy.is_shutdown():


        
#         st_pub.publish(hello_str)
#         th_pub.publish(hello_str)

#         rate.sleep()

if __name__ == '__main__':

    rospy.init_node('PID_publisher_node', anonymous=True)
    handle = LineFollower()
    handle.run_PID()

