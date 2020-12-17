#!/usr/bin/env python3

import os
import time
import rospy
import numpy as np
import cv2
from simple_pid import PID
from std_msgs.msg import Float64


class LineFollower:

    def __init__(self):
        rospy.loginfo('class initiated')
        #self.vert_scan_y = 300   # num pixels from the top to start horiz scan
        #self.vert_scan_height = 30 # num pixels high to grab from horiz scan
        self.horz_scan_x = 125
        self.horz_scan_width = 500
        self.color_thr_low = np.asarray((15, 50, 50)) # hsv dark yellow
        self.color_thr_hi = np.asarray((30, 255, 255)) # hsv light yellow
        self.target_pixel = None # of the N slots above, which is the ideal relationship target
        self.steering = 0.0 # from -1 to 1
        self.throttle = 0.05 # from -1 to 1
        self.recording = True # Set to true if desired to save camera frames
        self.delta_th = 0.05 # how much to change throttle when off
        self.throttle_max = 0.1
        self.throttle_min = 0.05
        self.pid_st = PID(Kp=-0.01, Ki=0.0, Kd=-0.001)
        # self.cam_sub = rospy.Subscriber("/camera_image",Image)
        self.st_pub = rospy.Publisher('/steering_value', Float64, queue_size=1)
        self.th_pub = rospy.Publisher('/throttle_value', Float64, queue_size=1)
        self.cam_obj = cv2.VideoCapture(0)

        


    def find_line(self, cam_img):

        # take a horizontal slice of the image
        #iSlice = self.vert_scan_y

        height, width, channels = cam_img.shape
        vert_scan_y = 150 # num pixels from the top to start horiz scan
        iSlice = vert_scan_y
        vert_scan_height = height - 100 # num pixels high to grab from horiz scan

        #scan_line = cam_img[iSlice : iSlice + self.vert_scan_height, :, :]
        scan_line = cam_img[iSlice : iSlice + vert_scan_height, 100:width-100, :]
        


        # convert to HSV color space
        img_hsv = cv2.cvtColor(scan_line, cv2.COLOR_RGB2HSV)

        # make a mask of the colors in our range we are looking for
        mask = cv2.inRange(img_hsv, self.color_thr_low, self.color_thr_hi)

        # which index of the range has the highest amount of yellow?
        hist = np.sum(mask, axis=0)
        max_yellow = np.argmax(hist)
        rospy.loginfo(max_yellow)

        return max_yellow, hist[max_yellow], mask

    def img_extract(self):
        rval, frame = self.cam_obj.read()
        if rval == True :
            rval, frame = self.cam_obj.read()
            
            path = '/home/jetson/projects/catkin_ws/src/cvai/data'

            #filename = 'savedImage.jpg'

            #cv2.imwrite(os.path.join(path, 'robocar_img_3.jpg'), frame)
            #rospy.loginfo('image taken!')
            # frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
            return frame
            self.cam_obj.release()

    def run_PID(self):

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            cam_img = self.img_extract()

            max_yellow, confidense, mask = self.find_line(cam_img)
            conf_thresh = 0.001
            #rospy.loginfo(confidense)

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

if __name__ == '__main__':
    rospy.init_node('PID_publisher_node', anonymous=True)
    obj = LineFollower()
    try:
        obj.run_PID()
    except rospy.ROSInterruptException:
        pass
    
