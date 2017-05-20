#!/usr/bin/env python

# --------------------------------------------------------------------------------------\n
# Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
# Modified by: Angshuman Goswami, Graduate Student, Clemson University
# Date Create: 20/5/2016, Last Modified: 20/5/2016 \n
# --------------------------------------------------------------------------------------\n

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from simulator.msg import ECU_raw, Z_DynBkMdl
from numpy import pi
import time
import rospy
from geometry_msgs.msg import Vector3
import math
from pid import PID
from numpy import sin, cos, tan, arctan, arcsin, array, dot
from numpy import sign, argmin, sqrt

str_ang     = 0
str_ang_max = 25
str_ang_min = -25
phi = 0
phi_d = 0
acc_d = 0
v_meas = 0

p 		= 1 #rospy.get_param("controller/p")
i 		= 0 #rospy.get_param("controller/i")
d 		= 0 #rospy.get_param("controller/d")
pid             = PID(P=p, I=i, D=d)

# for bypassing PID

read_ecu_time = False
ecu_time_diff = 0
ecu_time_prev = 0
d_f_d    = 0   
ecu_time = 5

def ecu_callback(msg):
    global acc_d, phi_d, read_ecu_time, ecu_time_diff, ecu_time_prev, d_f_d, ecu_time
    acc_d  = msg.motor
    d_f_d  = msg.servo
    phi_d  = msg.psi
    ecu_time = time.time()#rospy.get_rostime().secs + rospy.get_rostime().nsecs/float(10**9)
    if not read_ecu_time:
        ecu_time_prev = ecu_time
        read_ecu_time = True
    ecu_time_diff = ecu_time - ecu_time_prev
    ecu_time_prev = ecu_time
    
def state_estimate_callback(msg):
    global phi, phi_d, acc_d,b0, a, b
    global str_ang_max, str_ang_min,  str_ang
    global FxR, str_ang
    X      = msg.x
    Y      = msg.y
    phi    = msg.psi
    v_x    = msg.v_x
    v_y    = msg.v_y
    v_meas = math.sqrt(v_x**2 + v_y**2)
    #rospy.loginfo("%s",d_target)
    # translate from SI units in vehicle model
    # to pwm angle units (i.e. to send command signal to actuators)

    
    #Error in demand and actual phi
    phi_err = phi_d - phi
    #rospy.loginfo("%s",phi_err)
    rateHz  = 5
    dt      = 1.0 / rateHz
    
    u        = pid.update(phi_err, dt)
 
         
    #rospy.loginfo("%s",u)
    if   v_meas>0:
         bta   = -b*u/(dt*v_meas)
         #bta   = b*phi_err/(dt*v_meas)
         if bta > 0.225:
              bta = 0.225
         elif bta < - 0.225:
              bta = - 0.225
         d_f    = arctan((a+b)/a*tan(arcsin(bta)))
    else:
         d_f = 0
    #rospy.loginfo("%s",d_f)
    # convert desired steering angle to degrees, saturate based on input limits
    str_ang     = max( min( 180.0/pi*d_f, str_ang_max), str_ang_min)
    
    #rospy.loginfo("%s",str_ang)
    

    # compute motor command
    FxR         =  float(acc_d) 
    #rospy.loginfo("%s",FxR)

    update_arduino()



def update_arduino():
    global FxR, str_ang , str_ang, ecu_time_diff, d_f_d, ecu_time
    ecu_cmd = ECU_raw(FxR, str_ang*pi/180, ecu_time)  
    #ecu_cmd = ECU_raw(FxR, d_f_d, ecu_time)         #uncomment to deactive PID and comment out the previous line
    ecu_pub.publish(ecu_cmd)

def arduino_interface():
    global ecu_pub, b0, a, b

    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('veh1_pid')
    #Get input gain
    b0  =  0.3 #rospy.get_param("controller/input_gain")
    #Get vehicle parameters
    a = 0.125#rospy.get_param("state_estimation/L_a")       # distance from CoG to front axel
    b = 0.125#rospy.get_param("state_estimation/L_b")       # distance from CoG to rear axel

    Subscriber('ecu_1', ECU_raw, ecu_callback, queue_size = 10)
    Subscriber("z_vhcl_1", Z_DynBkMdl, state_estimate_callback, queue_size=10)
    ecu_pub = Publisher('ecu_cmd_1', ECU_raw, queue_size = 10)
    
    while not rospy.is_shutdown():
       
       # process callbacks and keep alive
       rospy.Rate(10).sleep()

#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
