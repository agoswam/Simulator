#!/usr/bin/env python

# --------------------------------------------------------------------------------------\n
# Author: Angshuman Goswami, Graduate Student, Clemson University
# Date Create: 20/5/2016, Last Modified: 20/5/2016 \n
# --------------------------------------------------------------------------------------\n

import rospy
import numpy
from simulator.msg import Z_DynBkMdl, ECU_raw
import datetime
import MySQLdb as mdb
from geometry_msgs.msg import Vector3


#for veh1
X         = 0
Y         = 0
psi       = 0
v_x       = 0
v_y       = 0
r         = 0
acc_d     = 0
steer_d   = 0

#for veh2
X2         = 0
Y2         = 2
psi2       = 0
v_x2       = 0
v_y2       = 0
r2         = 0
acc_d2     = 0
steer_d2   = 0

#for veh3
X3         = 0
Y3         = -5
psi3       = 0
v_x3       = 0
v_y3       = 0
r3         = 0
acc_d3     = 0
steer_d3   = 0


try:
  con = mdb.connect('130.127.199.212', 'root', 'Fr24clemson', 'autonomous');
  cur = con.cursor()
except:
    print "Error opening serial port."
    sys.exit(1)
       
    resp = ""
droptable1="DROP TABLE autonomous.veh1_sim"
droptable2="DROP TABLE autonomous.veh2_sim"
droptable3="DROP TABLE autonomous.veh3_sim"

cur.execute(droptable1)
cur.execute(droptable2)
cur.execute(droptable3)

createtable1="CREATE TABLE autonomous.veh1_sim( X DOUBLE NULL ,Y DOUBLE NULL, psi DOUBLE NULL, v_x DOUBLE NULL, v_y DOUBLE NULL  , r DOUBLE NULL, acc_d DOUBLE NULL, steer_d DOUBLE NULL, rostime DOUBLE NULL )"
createtable2="CREATE TABLE autonomous.veh2_sim( X DOUBLE NULL ,Y DOUBLE NULL, psi DOUBLE NULL, v_x DOUBLE NULL, v_y DOUBLE NULL  , r DOUBLE NULL, acc_d DOUBLE NULL, steer_d DOUBLE NULL, rostime DOUBLE NULL )"
createtable3="CREATE TABLE autonomous.veh3_sim( X DOUBLE NULL ,Y DOUBLE NULL, psi DOUBLE NULL, v_x DOUBLE NULL, v_y DOUBLE NULL  , r DOUBLE NULL, acc_d DOUBLE NULL, steer_d DOUBLE NULL, rostime DOUBLE NULL )"

cur.execute(createtable1)
cur.execute(createtable2)
cur.execute(createtable3)
                    
def veh1_pos_callback(data):
          
          global X, Y, psi, v_x, v_y, r

          
          X   = float(data.x)
          Y   = float(data.y)
          psi = float(data.psi)
          v_x = float(data.v_x)       
          v_y = float(data.v_y)
          r   = float(data.psi_dot)

def ecu1_callback(data):

          global acc_d, steer_d
          acc_d   = float(data.motor)
          steer_d = float(data.servo)


def veh2_pos_callback(data):
          
          global X2, Y2, psi2, v_x2, v_y2, r2

          
          X2   = float(data.x)
          Y2   = float(data.y)
          psi2 = float(data.psi)
          v_x2 = float(data.v_x)       
          v_y2 = float(data.v_y)
          r2   = float(data.psi_dot)

def ecu2_callback(data):

          global acc_d2, steer_d2
          acc_d2   = float(data.motor)
          steer_d2 = float(data.servo)

def veh3_pos_callback(data):
          
          global X3, Y3, psi3, v_x3, v_y3, r3

          
          X3   = float(data.x)
          Y3   = float(data.y)
          psi3 = float(data.psi)
          v_x3 = float(data.v_x)       
          v_y3 = float(data.v_y)
          r3   = float(data.psi_dot)

def ecu3_callback(data):

          global acc_d3, steer_d3
          acc_d3   = float(data.motor)
          steer_d3 = float(data.servo)


def mysql_server_data():
    global X, Y, psi, v_x, v_y, psi_dot, acc_d, steer_d
    global X2, Y2, psi2, v_x2, v_y2, psi_dot2, acc_d2, steer_d2
    global X3, Y3, psi3, v_x3, v_y3, psi_dot3, acc_d3, steer_d3
    rospy.init_node('mysql_server_data', anonymous=True)
    while not rospy.is_shutdown():    

     rospy.Subscriber('z_vhcl_1', Z_DynBkMdl, veh1_pos_callback)
     rospy.Subscriber('ecu_cmd_1', ECU_raw, ecu1_callback)
     rospy.Subscriber('z_vhcl_2', Z_DynBkMdl, veh2_pos_callback)
     rospy.Subscriber('ecu_cmd_2', ECU_raw, ecu2_callback)
     rospy.Subscriber('z_vhcl_3', Z_DynBkMdl, veh3_pos_callback)
     rospy.Subscriber('ecu_cmd_3', ECU_raw, ecu3_callback)
     
     rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs/float(10**9)
     sql1 = "insert into veh1_sim(X, Y, psi, v_x, v_y, r, acc_d, steer_d, rostime) values(%s, %s, %s, %s, %s, %s, %s, %s, %s)" % (X, Y, psi, v_x, v_y, r, acc_d, steer_d, rostime)
     sql2 = "insert into veh2_sim(X, Y, psi, v_x, v_y, r, acc_d, steer_d, rostime) values(%s, %s, %s, %s, %s, %s, %s, %s, %s)" % (X2, Y2, psi2, v_x2, v_y2, r2, acc_d2, steer_d2, rostime)
     sql3 = "insert into veh3_sim(X, Y, psi, v_x, v_y, r, acc_d, steer_d, rostime) values(%s, %s, %s, %s, %s, %s, %s, %s, %s)" % (X3, Y3, psi3, v_x3, v_y3, r3, acc_d3, steer_d3, rostime)
     print sql1
     print sql2
     print sql3
     cur.execute(sql1)
     cur.execute(sql2)
     cur.execute(sql3)
     print "Rows inserted: %s" % cur.rowcount
     con.commit()
     rospy.Rate(50)

if __name__=='__main__':
    try:
       mysql_server_data()
    except rospy.ROSInterruptException:
        pass

