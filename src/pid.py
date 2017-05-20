#!/usr/bin/env python

# --------------------------------------------------------------------------------------\n
# Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
# Modified by: Angshuman Goswami, Graduate Student, Clemson University
# Date Create: 20/5/2016, Last Modified: 20/5/2016 \n
# --------------------------------------------------------------------------------------\n
class PID:
    def __init__(self, P=2.0, I=0.0, D=0, de=0, e_int = 0, Integrator_max=50, Integrator_min=-50):
        self.Kp            = P             # proportional gain
        self.Ki             = I             # integral gain
        self.Kd           = D             # derivative gain

        self.set_point      = 0             # reference point
        self.e              = 0
        
        self.e_int          = 0
        self.int_e_max      = Integrator_max
        self.int_e_min      = Integrator_min

    def update(self,current_value, dt):
        e_t         = self.set_point - current_value
        de_t        = ( e_t - self.e)/dt
        
        if    -0.1 <e_t < 0.1:
                    self.e_int  = self.e_int + e_t * dt
                    if self.e_int > self.int_e_max:
                         self.e_int = self.int_e_max
                    elif self.e_int < self.int_e_min:
                         self.e_int = self.int_e_min
        else:
                    self.e_int = 0
            
        P_val  = self.Kp * e_t
        I_val  = self.Ki * self.e_int
        D_val  = self.Kd * de_t

        PID = P_val + I_val + D_val
        self.e      = e_t

        return PID

    def setPoint(self,set_point):
        self.set_point    = set_point
        self.e_int        = 0

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.e
  
#%% Example function
def fx(x, u, dt):
    x_next = x +  (3*x + u) * dt
    return x_next
  
#%% Test script to ensure program is functioning properly
if __name__ == "__main__":
    
    from numpy import zeros
    import matplotlib.pyplot as plt
    
    n       = 200
    x       = zeros(n)
    x[0]    = 20
    pid     = PID(P=3.7, I=5, D=0.5)
    dt = 0.1
    
    for i in range(n-1):
        u       = pid.update(x[i], dt)
        x[i+1] = fx(x[i],u, dt)
         
    plt.plot(x)
    plt.show()
