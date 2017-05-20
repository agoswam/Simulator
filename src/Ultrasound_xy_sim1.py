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
from barc.msg import Ultrasound, Z_DynBkMdl, Ultrasound_xy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from numpy import pi
import numpy as np
import scipy
from scipy import spatial
from numpy import arctan


sensor_r   = 5
Veh        = [0, 0,   0]
obs1       = [6, 0,   0]    
obs2       = [10, 1.5, 0]
obs3       = [14,0,   0]
obs4       = [18,-1.5,0]
obs5       = [9.5, -1.5, 0]  
obs6       = [14, 5, 0]  
obs7       = [19, 18, 0]  
obs8       = [10, 7, 0]
obs_angle  = 1.57   
psi        = 0



def veh1_pos_callback(data):
          
          global Veh, psi

          Veh   = [float(data.x), float(data.y), 0]
          psi   = float(data.psi) 
 
def ultrasound_xy():
        # initialize node
         rospy.init_node('ultrasound_xy', anonymous=True)
         state_pub1   = rospy.Publisher('ultrasound_xy', Ultrasound_xy, queue_size = 10)
         # set node rate
         loop_rate   = 50
         dt          = 1.0 / loop_rate
         rate        = rospy.Rate(loop_rate)
         t0          = time.time()
         while not rospy.is_shutdown():

               global Veh, psi
               #Weight on obstacles:
               Vehx = Veh[0]
               Vehy = Veh[1]
               a = np.matrix([obs1 , obs2, obs3, obs4, obs5, obs6, obs7, obs8])
               b = np.matrix(Veh)
               dist      = scipy.spatial.distance.cdist(a,b) # pick the appropriate distance metric 
               ind_dist  = sorted(range(len(dist)),key=lambda x:dist[x])  
               dis_1     = dist[ind_dist[0],:] 
               dis_2     = dist[ind_dist[1],:] 
               obs1x      = a[ind_dist[0],:2].tolist()[0][0]
               obs1y      = a[ind_dist[0],:2].tolist()[0][1]
               obs2x      = a[ind_dist[1],:2].tolist()[0][0]
               obs2y      = a[ind_dist[1],:2].tolist()[0][1]
               
               #if dis_1 < sensor_r:
               #     if dis_2 < sensor_r:
               #            (w1, w2) = (10**7, 10**7)
               #     else:
               #            (w1, w2) = (10**7, 0)
               #else:
               #            (w1, w2) = (0, 0)
               #slope1 = arctan((obs1y-Vehy)/(obs1x-Vehx))-psi
               #if not - obs_angle < slope1 < obs_angle:
               #     if - obs_angle < slope2 < obs_angle:
               #            (w1, w2) = (10**7, 10**7)
               #     else:
               #            (w1, w2) = (10**7, 0)
               #else:
               #            (w1, w2) = (0, 0)
               #     (obs1x, obs1y) = (obs2x, obs2y)
               obstacles_xy   = Ultrasound_xy(obs1x, obs1y, obs2x, obs2y, 10**3, 0)
               
               rospy.loginfo("%s", obstacles_xy)
               
               rospy.Subscriber('z_vhcl_1', Z_DynBkMdl, veh1_pos_callback)
               state_pub1.publish(obstacles_xy)
               rate.sleep()
    

    


if __name__=='__main__':
    try:
       ultrasound_xy()
    except rospy.ROSInterruptException:
        pass

