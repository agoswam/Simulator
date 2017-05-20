#!/usr/bin/env python

# --------------------------------------------------------------------------------------\n
# Author: Angshuman Goswami, Graduate Student, Clemson University
# Date Create: 20/5/2016, Last Modified: 20/5/2016 \n
# --------------------------------------------------------------------------------------\n

import rospy
import time
import os
import math
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from numpy import pi
from geometry_msgs.msg import Vector3

Target_X = 22
Target_Y = 0.1


def target():
        
     

        # initialize node
         global Target_X, Target_X
         rospy.init_node('target', anonymous=True)
         state_pub   = rospy.Publisher('target', Vector3, queue_size = 10)
         target_XY = Vector3(Target_X, Target_Y,0)
             # set node rate
         loop_rate   = 20
         dt          = 1.0 / loop_rate
         rate        = rospy.Rate(loop_rate)
         t0          = time.time()
         while not rospy.is_shutdown():
               state_pub.publish(target_XY)
               rate.sleep()
    

    


if __name__=='__main__':
    try:
       target()
    except rospy.ROSInterruptException:
        pass

