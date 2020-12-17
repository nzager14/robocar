#! /usr/bin/env python

import rospy     # Import the Python library for ROS
from std_msgs.msg import Int32 # Import the Int32 message from the std_msgs package
from pid_control import PID
#import donkeycar as dk
def __init__(self):
    setPoint_value = 0
    state_value = 0
   # self.pid_object = PID(setPoint_value, state_value)
    self.pid_st = PID(Kp=10, Ki=0.00, Kd=0.1)
    rospy.init_node('pid')
    pub = rospy.publisher('pid_topic',Int32,queue_size = 1)
    rate = rospy.Rate(2)
def callback(self,msg):
    cx = msg.data(1)
    self.steering = self.pid_st(cx)
   # self.pidpid_object.state_update(value=cx)
   # effort_value = self.pid_object.get_control_effort()
    rospy.logwarn("Effort Value=="+str(effort_value))
    pub.publish('self.steering')
    
rospy.init_node('pos_subscriber')                   # Initiate a Node called 'topic_subscriber'
    sub = rospy.Subscriber('posx', Int32, callback)   # Create a Subscriber object that will listen to the /counter
                                                      # topic and will cal the 'callback' function each time it reads
                                                      # something from the topic
rospy.spin() 
