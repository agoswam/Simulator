#!/usr/bin/env python

# --------------------------------------------------------------------------------------\n
# Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
# Modified by: Angshuman Goswami, Graduate Student, Clemson University
# Date Create: 20/5/2016, Last Modified: 20/5/2016 \n
# --------------------------------------------------------------------------------------\n

import rospy
import time
import math
from barc.msg import ECU_raw
from geometry_msgs.msg import Vector3
from simulator.msg import Z_DynBkMdl, eZ_DynBkMdl
from numpy import sin, cos, tan, arctan, array, dot, pi
from numpy import sign, argmin, sqrt, zeros
from system_models_simulator import f_KinBkMdl, f_DynBkMdl, ef_KinBkMdl, ef_DynBkMdl
# input variables
d_f         = 0
FxR         = 0
a              = 0
# raw measurement variables
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)

# from encoder
v_x_enc     = 0
t0          = time.time()
n_FL        = 0                     # counts in the front left tire
n_FR        = 0                     # counts in the front right tire
n_FL_prev   = 0
n_FR_prev   = 0
r_tire      = 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt      = 2.0*pi*r_tire/4.0     # distance along quarter tire edge
m           = 1.98                  # mass of the car
x_target    = 10
y_target    = 10

# for ECU 

read_ECU0 = False
read_veh2 = False

# for veh2

x2        = 0
y2        = 0





# target command update
def target_callback(data):

    global x_target, y_target
    x_target = data.x
    y_target = data.y
    #rospy.loginfo("%s",x_target)



# ecu1 command update
def ecu1_callback(data):
    global FxR, d_f
    FxR         = m*data.motor        # input acceleration
    d_f         = data.servo        # input steering angle

# ecu2 command update
def ecu2_callback(data):
    global read_veh2
    read_veh2 = True
    

# position of veh2
def veh2_pos_callback(data):
    global x2, y2
    x2 = data.x
    y2 = data.y


# state estimation node
def vehicle_simulator():

    global d_f, FxR, x_target, y_target, read_ECU0
    # initialize node
    rospy.init_node('vehicle_simulator', anonymous=True)
    
   

    # topic subscriptions / publications
    rospy.Subscriber('ecu_cmd_1', ECU_raw, ecu1_callback)
    rospy.Subscriber('ecu_2', ECU_raw, ecu2_callback)
    rospy.Subscriber('z_vhcl_2', Z_DynBkMdl, veh2_pos_callback)
    rospy.Subscriber('target', Vector3, target_callback)
    state_pub   = rospy.Publisher('z_vhcl_1', Z_DynBkMdl, queue_size = 10)
    state_pub2   = rospy.Publisher('v_sim_1', Vector3, queue_size = 10)
    #state_pub3   = rospy.Publisher('steering_sim', Vector3, queue_size = 10)
    #state_error_frame_pub   = rospy.Publisher('ez_vhcl', eZ_DynBkMdl, queue_size = 10)

        # get vehicle dimension parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("m")         # mass of vehicle
    I_z = rospy.get_param("I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get tire model
    B     = rospy.get_param("tire_model/B")
    C     = rospy.get_param("tire_model/C")
    mu    = rospy.get_param("tire_model/mu")
    trMdl = ([B,C,mu],[B,C,mu])

    # get external force model
    a0    = rospy.get_param("air_drag_coeff")
    Ff    = rospy.get_param("friction")
    F_ext = (a0, Ff)

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # set initial conditions 
    x   = 0
    y   = 0
    psi = 0
    v_x = 0
    v_y = 0
    r   = 0
    d_f_sens = 0

    s    = 0
    ey   = 0
    epsi = 0
    while not rospy.is_shutdown():

        # publish state estimate
        bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )
        d_target    = math.sqrt((x_target-x)**2 + (y_target-y)**2)
        
        if d_target<3:
          (FxR, d_f) = (0, 0)
          (v_x, v_y) = (0, 0)
        #if not read_ECU0:
         #   d_f_prev  = d_f
          #  FxR_prev  = FxR
           # read_ECU0 = True 
        #d_f_diff = d_f_prev-d_f
        #d_f_prev = d_f
        
       # FxR_diff = FxR_prev-FxR
        #FxR_prev = FxR

        #if d_f_diff == 0 and FxR_diff == 0:
         #   d_f = 0
          #  FxR = 0
        

        #to ensure the veh1 starts after veh2 opt completes (Uncomment if you want to disconnect veh2 communication)
        #if not read_veh2:
        #   d_f = 0
        #   FxR = 0
        #dis = math.sqrt((x2-x)**2 + (y2-y)**2)
        #if dis > 10:
         #   d_f = 0
          #  FxR = 0



        if abs(v_x) > 0.05:
            z  = (x,  y,  psi, v_x, v_y, r)
            ze = (s, ey, epsi, v_x, v_y, r)
            u = (d_f, FxR)
            d_f_sens = d_f
            
            (x, y,   psi, v_x, v_y, r) = f_DynBkMdl(z, u, vhMdl, trMdl, F_ext, dt)
            (s, ey, epsi, v_x, v_y, r) = ef_DynBkMdl(ze, u, vhMdl, trMdl, F_ext, dt)
            v = math.sqrt(v_x**2 + v_y**2)

        else:
            z  = (x, y,   psi, v_x)
            ze = (s, ey, epsi, v_x)
            u = (d_f, FxR)

            (x, y,   psi, v_x) = f_KinBkMdl(z,u, (L_a, L_b), F_ext, dt)
            (s, ey, epsi, v_x) = ef_KinBkMdl(ze,u, (L_a, L_b), F_ext, dt)
            v_y     = 0
            r = 0
            v = v_x
 
        
        # publish information
        #if v_x == 0:
        #        v_x = 0.0001
        state_pub.publish( Z_DynBkMdl(x, y, psi, v_x, v_y, r) )
        state_pub2.publish( Vector3(v, 0, 0) ) 
        #state_pub3.publish( Vector3(d_f_sens, 0, 0) )
        
        #state_error_frame_pub.publish( eZ_DynBkMdl(s, ey, epsi, v_x, v_y, r) )
        
        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       vehicle_simulator()
    except rospy.ROSInterruptException:
        pass
