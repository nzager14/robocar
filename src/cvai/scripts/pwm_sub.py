#!/usr/bin/env python3

#import Adafruit_PCA9685
import rospy 
import numpy as np
from std_msgs.msg import Float64 
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
pca.frequency = 60









def callback(msg):
    steer = (msg.data + 1)*90
    print(steer)
    kit.servo[0].angle = steer
    kit.continuous_servo[1].throttle = .05





rospy.init_node('pwm_sub')
sub = rospy.Subscriber('/steering_value', Float64, callback)
rospy.spin() 
