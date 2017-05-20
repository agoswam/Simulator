#!/usr/bin/env python

# --------------------------------------------------------------------------------------\n
# Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
# Modified by: Angshuman Goswami, Graduate Student, Clemson University
# Date Create: 20/5/2016, Last Modified: 20/5/2016 \n
# --------------------------------------------------------------------------------------\n

from numpy import sin, cos, tan, arctan, array, dot
from numpy import sign, argmin, sqrt, abs, pi
import rospy

# discrete non-linear bicycle model dynamics 6-dof
def f_DynBkMdl(z, u, vhMdl, trMdl, F_ext, dt): 
    """
    process model
    input: state z at time k, z[k] := [X[k], Y[k], psi[k], v_x[k], v_y[k], r[k]])
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    X       = z[0]
    Y       = z[1]
    psi     = z[2]
    v_x     = z[3]
    v_y     = z[4]
    r       = z[5]

    d_f     = u[0]
    FxR     = u[1]

    # extract parameters
    (a,b,m,I_z)             = vhMdl
    (a0, Ff)                = F_ext
    (trMdlFront, trMdlRear) = trMdl
    (B,C,mu)                = trMdlFront
    g                       = 9.81
    Fn                      = m*g/2.0         # assuming a = b (i.e. distance from CoG to either axel)

    # limit force to tire friction circle
    if FxR >= mu*Fn:
        FxR = mu*Fn

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    a_F     = arctan((v_y + a*r)/v_x) - d_f
    a_R     = arctan((v_y - b*r)/v_x)

    # compute lateral tire force at the front
    TM_param    = [B, C, mu*Fn]
    FyF         = -f_pajecka(TM_param, a_F)

    # compute lateral tire force at the rear
    # ensure that magnitude of longitudinal/lateral force lie within friction circle
    FyR_paj     = -f_pajecka(TM_param, a_R)
    FyR_max     = sqrt((mu*Fn)**2 - FxR**2)
    Fy          = array([FyR_max, FyR_paj])
    idx         = argmin(abs(Fy))
    FyR         = Fy[idx]

    # compute next state
    X_next      = X + dt*(v_x*cos(psi) - v_y*sin(psi)) 
    Y_next      = Y + dt*(v_x*sin(psi) + v_y*cos(psi)) 
    psi_next    = psi + dt*r
    v_x_next    = v_x + dt*(r*v_y +1/m*(FxR - FyF*sin(d_f)) - (a0*v_x**2 + Ff)*sign(v_x) )
    #v_x_next    = v_x + dt*(r*v_y +1/m*(FxR - FyF*sin(d_f)))
    v_y_next    = v_y + dt*(-r*v_x +1/m*(FyF*cos(d_f) + FyR))
    r_next      = r    + dt/I_z*(a*FyF*cos(d_f) - b*FyR)

    return array([X_next, Y_next, psi_next, v_x_next, v_y_next, r_next])
    
def ef_DynBkMdl(z, u, vhMdl, trMdl, F_ext, dt): 
    """
    process model
    input: state z at time k, z[k] := [S[k], Ey[k], epsi[k], v_x[k], v_y[k], r[k]])
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    S       = z[0]
    Ey      = z[1]
    epsi    = z[2]
    v_x     = z[3]
    v_y     = z[4]
    r       = z[5]

    d_f     = u[0]
    FxR     = u[1]

    # extract parameters
    (a,b,m,I_z)             = vhMdl
    (a0, Ff)                = F_ext
    (trMdlFront, trMdlRear) = trMdl
    (B,C,mu)                = trMdlFront
    g                       = 9.81
    Fn                      = m*g/2.0         # assuming a = b (i.e. distance from CoG to either axel)

    # limit force to tire friction circle
    if FxR >= mu*Fn:
        FxR = mu*Fn

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    a_F     = arctan((v_y + a*r)/v_x) - d_f
    a_R     = arctan((v_y - b*r)/v_x)

    # compute lateral tire force at the front
    TM_param    = [B, C, mu*Fn]
    FyF         = -f_pajecka(TM_param, a_F)

    # compute lateral tire force at the rear
    # ensure that magnitude of longitudinal/lateral force lie within friction circle
    FyR_paj     = -f_pajecka(TM_param, a_R)
    FyR_max     = sqrt((mu*Fn)**2 - FxR**2)
    Fy          = array([FyR_max, FyR_paj])
    idx         = argmin(abs(Fy))
    FyR         = Fy[idx]

    # compute next state
    curv        = f_curvature(S)

    dSdt        = (v_x*cos(epsi) - v_y*sin(epsi) ) / ( 1 - Ey * curv)
    
    S_next      = S + dt*dSdt
    Ey_next     = Ey + dt*(v_x*sin(epsi) + v_y*cos(epsi)) 
    epsi_next   = epsi + dt*(r - curv * dSdt )
    v_x_next    = v_x + dt*(r*v_y +1/m*(FxR - FyF*sin(d_f)) - (a0*v_x**2 + Ff)*sign(v_x) )
    v_y_next    = v_y + dt*(-r*v_x +1/m*(FyF*cos(d_f) + FyR))
    r_next      = r    + dt/I_z*(a*FyF*cos(d_f) - b*FyR)

    return array([S_next, Ey_next, epsi_next, v_x_next, v_y_next, r_next])   

def f_pajecka(trMdl, alpha):
    """
    f_pajecka = d*sin(c*atan(b*alpha))    
    
    inputs :
        * trMdl := tire model, a list or tuple of parameters (b,c,d)
        * alpha := tire slip angle [radians]
    outputs :
        * Fy := lateral force from tire [Newtons]
    """
    (b,c,d) = trMdl
    return  d*sin(c*arctan(b*alpha)) 

def f_curvature(s):
    """
    f_curvature = curvature of the road at the curvilinear abscissa s    
    
    inputs :
        * s := curvilinear absicssa
    outputs :
        * curv := road curvature 
    """
    s_start_curve = 20.0
    curvature_rad = 10.0
    if (s > s_start_curve) and (s < (s_start_curve + pi * curvature_rad / 2.0)):
        curv = 1.0/curvature_rad
    else:
        curv = 0.0

    return curv

def f_KinBkMdl(z,u,vhMdl, F_ext, dt):
    """
    process model
    input: state z at time k, z[k] := [x[k], y[k], psi[k], v[k]]
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    x       = z[0]
    y       = z[1]
    psi     = z[2]
    v       = z[3]

    d_f     = u[0]
    a       = u[1]

    # extract parameters
    (L_a, L_b)             = vhMdl
    (a0, Ff)               = F_ext

    # compute slip angle
    bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

    # compute next state
    x_next      = x + dt*( v*cos(psi + bta) ) 
    y_next      = y + dt*( v*sin(psi + bta) ) 
    psi_next    = psi + dt*v/L_b*sin(bta)
    v_next      = v + dt*(a  - (a0*v**2 + Ff)*sign(v) )

    if abs(v_next) < 0.0005:
        v_next = 0

    return array([x_next, y_next, psi_next, v_next])

def ef_KinBkMdl(z,u,vhMdl, F_ext, dt):
    """
    process model
    input: state z at time k, z[k] := [x[k], y[k], epsi[k], v[k]]
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    s       = z[0]
    ey      = z[1]
    epsi    = z[2]
    v       = z[3]

    d_f     = u[0]
    a       = u[1]


    # extract parameters
    (L_a, L_b)             = vhMdl
    (a0, Ff)               = F_ext

    # compute slip angle
    bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

    # compute next state
    curv        = f_curvature(s)

    dsdt        = v*cos(epsi) / ( 1 - ey * curv)

    s_next      = s + dt*dsdt 
    ey_next     = ey + dt*( v*sin(epsi + bta) ) 
    epsi_next   = epsi + dt*(v/L_b*sin(bta) - curv *dsdt)
    v_next      = v + dt*(a  - (a0*v**2 + Ff)*sign(v) )

    if abs(v_next) < 0.0005:
        v_next = 0

    return array([s_next, ey_next, epsi_next, v_next])
 
