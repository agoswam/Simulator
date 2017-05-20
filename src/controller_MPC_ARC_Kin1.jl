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
dt      = 0.2              # time step of system

# preview horizon
N       =  12

# define targets [generic values]
#x_ref   = 10
#y_ref   = 10
#w       = sqrt(x_ref^2+y_ref^2)

# define decision variables 
# states: position (x,y), yaw angle, and velocity
# inputs: acceleration, steering angle 
println("Creating kinematic bicycle model ....")
mdl     = Model(solver = IpoptSolver(print_level=3))
#mdl     = Model(solver = BonminNLSolver())
#mdl     = Model(solver = NLoptSolver(algorithm=:LD_MMA))
@defVar( mdl, x[1:(N+1)] )
@defVar( mdl, y[1:(N+1)] )
@defVar( mdl, psi[1:(N+1)] )
@defVar( mdl, v[1:(N+1)] )
@defVar( mdl, r[1:N] )
@defVar( mdl, obsx1[1:N+1] )
@defVar( mdl, obsy1[1:N+1] )
#@defVar( mdl, obsx2[1:N+1] )
#@defVar( mdl, obsy2[1:N+1] )
@defVar( mdl, a[1:N] )
@defVar( mdl, targetx[1:N+1] )
@defVar( mdl, targety[1:N+1] )
@defVar( mdl, epsi1[1:N+1] )
#@defVar( mdl, epsi2[1:N+1] )
@defVar( mdl, w1[1:N+1] )
@defVar( mdl, w2[1:N+1] )



# define objective function
@setNLObjective(mdl, Min, 1*(x[N+1] - targetx[1])^2 + 1*(y[N+1] - targety[1])^2 + 0.1*((r[N])^2 + (r[N-1])^2 + (r[N-2])^2 + (r[N-3])^2 + (r[N-4])^2 + (r[N-5])^2 + (r[N-6])^2 + (r[N-7])^2 + (r[N-8])^2 + (r[N-9])^2 + (r[N-10])^2 + (r[N-11])^2)  + 10^4*((epsi1[N+1]) + (epsi1[N]) + (epsi1[N-1]) + (epsi1[N-2]) + (epsi1[N-3]) + (epsi1[N-4]) + (epsi1[N-5]) + (epsi1[N-6]) + (epsi1[N-7]) + (epsi1[N-8]) + (epsi1[N-9]) + (epsi1[N-10]) + (epsi1[N-11])))

# define constraints
# define system dynamics
# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
#               Spring, 2011, page 26
@defNLParam(mdl, x0                   == 0);              @addNLConstraint(mdl, x[1]                  == x0);
@defNLParam(mdl, y0                   == 0);              @addNLConstraint(mdl, y[1]                  == y0);
@defNLParam(mdl, psi0                 == 0);              @addNLConstraint(mdl, psi[1]                == psi0 );
@defNLParam(mdl, v0                   == 0);              @addNLConstraint(mdl, v[1]                  == v0);
@defNLParam(mdl, obsx10               == 20);             @addNLConstraint(mdl, obsx1[1]              == obsx10);
@defNLParam(mdl, obsy10               == 0);              @addNLConstraint(mdl, obsy1[1]              == obsy10);
#@defNLParam(mdl, obsx20               == 20);             @addNLConstraint(mdl, obsx2[1]              == obsx20);
#@defNLParam(mdl, obsy20               == 0);              @addNLConstraint(mdl, obsy2[1]              == obsy20);
@defNLParam(mdl, targetx0             == 10);             @addNLConstraint(mdl, targetx[1]            == targetx0);
@defNLParam(mdl, targety0             == 10);             @addNLConstraint(mdl, targety[1]            == targety0);
@defNLParam(mdl, w10                  == 10);             @addNLConstraint(mdl, w1[1]                 == w10);
@defNLParam(mdl, w20                  == 10);             @addNLConstraint(mdl, w2[1]                 == w20);


@defNLExpr(mdl, obs1[i=1:N+1]       ,            (obsx1[i]-x[i])^2      + (obsy1[i]-y[i])^2)
#@defNLExpr(mdl, obs2[i=1:N+1]       ,            (obsx2[i]-x[i])^2      + (obsy2[i]-y[i])^2)

for i in 1:N
    @addNLConstraint(mdl, x[i+1]                                 == x[i]      + dt*(v[i]*cos( psi[i] )  ) )
    @addNLConstraint(mdl, y[i+1]                                 == y[i]      + dt*(v[i]*sin( psi[i] )  ) )
    @addNLConstraint(mdl, psi[i+1]                               == psi[i]    + dt*(r[i])  )
    @addNLConstraint(mdl, v[i+1]                                 == v[i]      + dt*(a[i])  )
    @addNLConstraint(mdl, obsx1[i+1]                             == obsx1[i])
    @addNLConstraint(mdl, obsy1[i+1]                             == obsy1[i])
    #@addNLConstraint(mdl, obsx2[i+1]                             == obsx2[i])
    #@addNLConstraint(mdl, obsy2[i+1]                             == obsy2[i])
    @addNLConstraint(mdl, w1[i+1]                                == w1[i])
    @addNLConstraint(mdl, w2[i+1]                                == w2[i])

    

    @addNLConstraint(mdl, targetx[i+1]                           == targetx[i])
    @addNLConstraint(mdl, targety[i+1]                           == targety[i])
    @addNLConstraint(mdl, epsi1[i+1]                             == epsi1[i] + dt*(epsi1[i]))
    #@addNLConstraint(mdl, epsi2[i+1]                             == epsi2[i])


    @addNLConstraint(mdl,  0              <= a[i]                      <=1)
    @addNLConstraint(mdl,  0              <= v[i]                      <=1.5)
    @addNLConstraint(mdl, -2.7            <= r[i]                      <=2.7)
        
    @addNLConstraint(mdl, epsi1[i] >=0)
    #@addNLConstraint(mdl, epsi2[i] >=0)

    @addNLConstraint(mdl, obs1[i] + epsi1[i] >=3)
    #@addNLConstraint(mdl, obs2[i] + epsi2[i] >=4)

    

end

# status update
println("initial solve ...")
status = solve(mdl)
println("finished initial solve!")

function state_estimate_callback(msg::Z_DynBkMdl)
    # update mpc 
    setValue(x0,     msg.x)
    setValue(y0,     msg.y)
    setValue(psi0,   msg.psi) 
    
end

function velocity_callback(msg::Vector3)
    # update mpc 
    setValue(v0,     msg.x)

end


function ultrasound1_xy_callback(msg::Ultrasound_xy)
    # update obstacles
    setValue(obsx10             ,    msg.frontx)
    setValue(obsy10             ,    msg.fronty)
    #setValue(obsx20             ,    msg.frontleftx)
    #setValue(obsy20             ,    msg.frontylefty)
    setValue(w10                ,    msg.frontrightx)
    #setValue(w20                ,    msg.frontrighty)
    
    
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
    s2  = Subscriber("v_sim_1", Vector3, velocity_callback, queue_size=10)
    s3  = Subscriber("ultrasound_xy", Ultrasound_xy, ultrasound1_xy_callback, queue_size=10)
    s4  = Subscriber("target", Vector3, target_callback, queue_size=10)
    loop_rate = Rate(5)
    counter = 0
    while ! is_shutdown()
        # run mpc, publish command
	tic()
        status = solve(mdl)
        toc()
        
        # get optimal solutions
        a_opt   = getValue(a[1])
 
        r_opt   = getValue(r[1])
        
        if( status == :Infeasible)
           a_opt = 0
        end

        psi_opt = getValue(psi[2])

        obsx1_opt   = getValue(obsx1[1])
        obsy1_opt   = getValue(obsy1[1])
        targetx_opt = getValue(targetx[1])
        targety_opt = getValue(targety[1])
        x_opt       = getValue(epsi1[1])
        y_opt       = getValue(epsi1[2])
        v_opt       = getValue(epsi1[3])
        epsi1_opt   = getValue(epsi1[4])
        # to ensure that the MPC runs multiple iteration before the car moves to get rid of intial infeasible results
        counter += 1
        if counter > 75
            # publish commands
              
              cmd = ECU_raw(a_opt, r_opt,psi_opt)
              publish(pub, cmd)
              
              #for debugging
              debug1 = Z_DynBkMdl(x_opt, y_opt, v_opt, obsx1_opt, obsy1_opt,epsi1_opt)
              publish(pub1, debug1)
        end
        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
