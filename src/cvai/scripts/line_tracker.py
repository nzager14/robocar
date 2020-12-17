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
        rospy.loginfo('LineFollower class initiated')
        self.color_thr_low = np.asarray((0, 0, 150)) # hsv dark yellow
        self.color_thr_hi = np.asarray((255,50 , 255)) # hsv light yellow
        self.target_pixel = None # of the N slots above, which is the ideal relationship target
        self.steering = 0.0 # from -1 to 1
        self.throttle = 0.95# from -1 to 1
        self.recording = True # Set to true if desired to save camera frames
        self.delta_th = 0.05 # how much to change throttle when off
        self.throttle_max = 0.3
        self.throttle_min = 0.02
        self.pid_st = PID(Kp=-0.0015, Ki=0.0000, Kd=-0.0002)
        self.st_pub = rospy.Publisher('/steering_value', Float64, queue_size=1)
        self.th_pub = rospy.Publisher('/throttle_value', Float64, queue_size=1)
        self.cam_obj = cv2.VideoCapture(0)

        


    def find_line(self, cam_img):

        height, width, channels = cam_img.shape

        vert_scan_y = height-200  # num pixels from the top to start horiz scan
        vert_scan_height = 50 # num pixels high to grab from horiz scan
        iSlice = vert_scan_y
        #iSlice + vert_scan_height
        scan_line = cam_img[iSlice :iSlice + 100, :width, :]

        h2, w2, ch2 = scan_line.shape
        # convert to HSV color space
        img_hsv = cv2.cvtColor(scan_line, cv2.COLOR_RGB2HSV)

        # make a mask of the colors in our range we are looking for
        mask = cv2.inRange(img_hsv, self.color_thr_low, self.color_thr_hi)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = h2/2, w2/2
        
        setPoint_value = w2/2
        
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)

        path = '/home/jetson/projects/catkin_ws/src/cvai/data'

        #filename = 'savedImage.jpg'

        #cv2.imwrite(os.path.join(path, 'robocar_img_10.jpg'), mask)




        return int(setPoint_value), int(cx)


        '''
        # which index of the range has the highest amount of yellow?
        hist = np.sum(mask, axis=0)
        max_yellow = np.argmax(hist)
        '''
        # return max_yellow, hist[max_yellow], mask


    def img_extract(self):

        rval, frame = self.cam_obj.read()
        if rval == True :
            
            rval, frame = self.cam_obj.read()
            #rospy.loginfo('image taken!')

            return frame
            self.cam_obj.release()

    def run_PID(self):

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            
            cam_img = self.img_extract()

            setPoint_value, cx = self.find_line(cam_img)
            # conf_thresh = 0.001
            
            self.pid_st.output_limits = (-600,600)

            self.pid_st.setpoint = setPoint_value
            self.steering = self.pid_st(cx)

            # self.steering = self.pid_object.get_control_effort()

            #rospy.logwarn("Set Value=="+str(setPoint_value))
            #rospy.logwarn("State Value=="+str(cx))
            #rospy.logwarn("Effort Value=="+str(self.steering))

            self.st_pub.publish(self.steering)
            self.th_pub.publish(self.throttle)
            
            if rospy.is_shutdown():
                self.th_pub.publish(0)

            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('PID_publisher_node', anonymous=True)
    obj = LineFollower()
    try:
        obj.run_PID()
    except rospy.ROSInterruptException:
        pass
    
