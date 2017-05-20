#!/usr/bin/env julia

#=
--------------------------------------------------------------------------------------\n
Original Authors: BARC Project, Berkely MPC Laboratory -> https://github.com/MPC-Berkeley/barc
Modified by: Angshuman Goswami, Graduate Student, Clemson University
Date Create: 20/5/2016, Last Modified: 20/5/2016 \n
--------------------------------------------------------------------------------------\n
=#

using RobotOS
@rosimport barc.msg: ECU_raw, Ultrasound_xy, Z_DynBkMdl
#@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
#using data_service.msg
using geometry_msgs.msg
using JuMP
using Ipopt
#using AmplNLWriter
#using NLopt

# define model parameters
L_a     = 0.125         # distance from CoG to front axel
L_b     = 0.125         # distance from CoG to rear axel
dt      = 0.2           # time step of system
Ff      = 0.1711        # friction coefficient
a0     = 0.1308         # air drag coeff
a       =  0.125        # distance from CoG to rear axel
b       =  0.125        # distance from CoG to front axel
I_z    =  0.24          # moment of Inertia
m     =  1.98           # mass of the vehicle

# Take the inverse due to issue with derivatives

m   = 1/m
I_z = 1/I_z

# Tire Properties
TM_F        = [10, 1.9, 1]
TM_R        = [10, 1.9, 1]
(B1,C1,D1)  = TM_F
(B2,C2,D2)  = TM_R



# preview horizon
N       =  12

# define targets [generic values]
#x_ref   = 10
#y_ref   = 10
#w       = sqrt(x_ref^2+y_ref^2)

# define decision variables 
# states: position (x,y), yaw angle, and velocity
# inputs: acceleration, steering angle 
println("Creating dynamic bicycle model ....")
mdl     = Model(solver = IpoptSolver(print_level=3))
#mdl     = Model(solver = BonminNLSolver())
#mdl     = Model(solver = NLoptSolver(algorithm=:LD_MMA))
@defVar( mdl, x[1:(N+1)] )
@defVar( mdl, y[1:(N+1)] )
@defVar( mdl, phi[1:(N+1)] )
@defVar( mdl, v_x[1:(N+1)] )
@defVar( mdl, v_y[1:(N+1)] )
@defVar( mdl, r[1:(N+1)] )
@defVar( mdl, delta[1:(N)] )
@defVar( mdl, FxR[1:(N)] )
#@defVar( mdl, v_x_inv[1:(N+1)] )

@defVar( mdl, obsx1[1:N+1] )
@defVar( mdl, obsy1[1:N+1] )
@defVar( mdl, targetx[1:N+1] )
@defVar( mdl, targety[1:N+1] )
@defVar( mdl, ephi1[1:N+1] )
@defVar( mdl, w1[1:N+1] )



# define objective function
@setNLObjective(mdl, Min, 1*(x[N+1] - targetx[1])^2 + 1*(y[N+1] - targety[1])^2 + 1*((delta[N])^2 + 1*(delta[N-1])^2 + (delta[N-2])^2 + (delta[N-3])^2 + (delta[N-4])^2 + (delta[N-5])^2 + (delta[N-6])^2 + (delta[N-7])^2)  + 10^3*(ephi1[N+1]) )

# define constraints
# define system dynamics
# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
#               Spring, 2011, page 26
@defNLParam(mdl, x0                   == 0);              @addNLConstraint(mdl, x[1]                  == x0);
@defNLParam(mdl, y0                   == 0);              @addNLConstraint(mdl, y[1]                  == y0);
@defNLParam(mdl, phi0                 == 0);              @addNLConstraint(mdl, phi[1]                == phi0 );
@defNLParam(mdl, v_x0                 == 0.1);            @addNLConstraint(mdl, v_x[1]                == v_x0);
@defNLParam(mdl, v_y0                 == 0);              @addNLConstraint(mdl, v_y[1]                == v_y0);
@defNLParam(mdl, r0                   == 0);              @addNLConstraint(mdl, r[1]                  == r0);
@defNLParam(mdl, obsx10               == 20);             @addNLConstraint(mdl, obsx1[1]              == obsx10);
@defNLParam(mdl, obsy10               == 0);              @addNLConstraint(mdl, obsy1[1]              == obsy10);
@defNLParam(mdl, targetx0             == 10);             @addNLConstraint(mdl, targetx[1]            == targetx0);
@defNLParam(mdl, targety0             == 10);             @addNLConstraint(mdl, targety[1]            == targety0);
#@defNLParam(mdl, w10                  == 10);             @addNLConstraint(mdl, w1[1]                 == w10);


# Compute distance to obstacle
@defNLExpr(mdl, obs1[i=1:N+1]       ,            (obsx1[i]-x[i])^2      + (obsy1[i]-y[i])^2)
# Compute magnitude of velocity
@defNLExpr(mdl, v[i=1:N+1]          ,            (v_x[i])^2      + (v_y[i])^2)
# Compute lateral forces from slip angles ang  and  pajecka tire model
@defNLExpr(mdl, a_F[i = 1:N],   atan((v_y[i] + a*r[i])/(v_x[i] + 0.001)) - delta[i])
@defNLExpr(mdl, a_R[i = 1:N],   atan((v_y[i] - b*r[i])/(v_x[i] + 0.001) )
@defNLExpr(mdl, FyF[i = 1:N],   D1*sin(C1*atan(B1 * a_F[i])) )
@defNLExpr(mdl, FyR[i = 1:N],   D2*sin(C2*atan(B2 * a_R[i])) )


for i in 1:N
    @addNLConstraint(mdl, x[i+1]                        == x[i]    + dt*(v_x[i]*cos(phi[i]) - v_y[i]*sin(phi[i]))             )
    @addNLConstraint(mdl, y[i+1]                        == y[i]    + dt*(v_x[i]*sin(phi[i])  + v_y[i]*cos(phi[i]))            )
    @addNLConstraint(mdl, phi[i+1]                      == phi[i]  + dt*r[i]                                                  )
    @addNLConstraint(mdl, v_x[i+1]                      == v_x[i]  + dt*(r[i]  *v_y[i]  + (FxR[i] - FyF[i]*sin(delta[i]))*m)  )
    @addNLConstraint(mdl, v_y[i+1]                      == v_y[i]  + dt*(-r[i] *v_x[i]  + (FyF[i]*cos(delta[i]) +   FyR[i])*m))
    @addNLConstraint(mdl, r[i+1]                        == r[i]    + dt*(a*FyF[i]*cos(delta[i]) - b*FyR[i])*I_z)
    @addNLConstraint(mdl, obsx1[i+1]                    == obsx1[i])
    @addNLConstraint(mdl, obsy1[i+1]                    == obsy1[i])

    #@addNLConstraint(mdl, w1[i+1]                                == w1[i])

    

    @addNLConstraint(mdl, targetx[i+1]                           == targetx[i])
    @addNLConstraint(mdl, targety[i+1]                           == targety[i])
    @addNLConstraint(mdl, ephi1[i+1]                             == ephi1[i])
    

    @addNLConstraint(mdl,  0              <= FxR[i]                    <=1)
    @addNLConstraint(mdl,  0              <= v[i]                      <=2.5)
    @addNLConstraint(mdl, -0.5            <= delta[i]                  <=0.5)
    @addNLConstraint(mdl, -2.7            <= r[i]                      <=2.7)

        
    @addNLConstraint(mdl, ephi1[i] >=0)
    
    @addNLConstraint(mdl, obs1[i] + ephi1[i] >=4)
    
    

end

# status update
println("initial solve ...")
status = solve(mdl)
println("finished initial solve!")

function state_estimate_callback(msg::Z_DynBkMdl)
    # update mpc 
    setValue(x0,       msg.x)
    setValue(y0,       msg.y)
    setValue(phi0,     msg.psi) 
    setValue(v_x0,     msg.v_x)
    setValue(v_y0,     msg.v_y)
    setValue(r0,       msg.psi_dot)
    
end

function ultrasound1_xy_callback(msg::Ultrasound_xy)
    # update obstacles
    setValue(obsx10             ,    msg.frontx)
    setValue(obsy10             ,    msg.fronty)
    #setValue(v_x_inv            ,    msg.frontrightx)
    
    #setValue(w10                ,    msg.frontrightx)
    
    
end   


function target_callback(msg::Vector3)
    # update target 
    setValue(targetx0             ,    msg.x)
    setValue(targety0             ,    msg.y)
    
    
   
end




function main()
    # initiate node, set up publisher / subscriber topics
    init_node("mpc1")
    pub = Publisher("ecu_1", ECU_raw, queue_size=10)
    pub1= Publisher("debug1_MPC", Z_DynBkMdl, queue_size=10)
    s1  = Subscriber("z_vhcl_1", Z_DynBkMdl, state_estimate_callback, queue_size=10)
    s3  = Subscriber("ultrasound_xy", Ultrasound_xy, ultrasound1_xy_callback, queue_size=10)
    s4  = Subscriber("target", Vector3, target_callback, queue_size=10)
    loop_rate = Rate(10)
    counter = 0
    while ! is_shutdown()
        # run mpc, publish command
	tic()
        status = solve(mdl)
        toc()
        
        # get optimal solutions
        FxR_opt   = getValue(FxR[1])
 
        delta_opt = getValue(delta[1])
        
        if( status == :Infeasible)
           FxR_opt = 0
        end

        phi_opt = getValue(phi[2])

        obsx1_opt   = getValue(FyF[1])
        obsy1_opt   = getValue(FyR[1])
        targetx_opt = getValue(targetx[1])
        targety_opt = getValue(targety[1])
        x_opt       = getValue(ephi1[1])
        y_opt       = getValue(ephi1[2])
        v_opt       = getValue(ephi1[3])
        ephi1_opt   = getValue(ephi1[4])

        # to ensure that the MPC runs multiple iteration before the car moves to get rid of intial infeasible results
        counter += 1
        if counter > 75
            # publish commands
              
              cmd = ECU_raw(FxR_opt, delta_opt,phi_opt)
              publish(pub, cmd)
              
              #for debugging
              debug1 = Z_DynBkMdl(x_opt, y_opt, v_opt, obsx1_opt, obsy1_opt,ephi1_opt)
              publish(pub1, debug1)
        end
        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
