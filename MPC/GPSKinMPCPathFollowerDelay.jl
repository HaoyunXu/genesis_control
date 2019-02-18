#!/usr/bin/env julia

#=
 This is a modified version of controller_MPC.jl from the barc project ROS codebase.
 Modified by Vijay Govindarajan.

 Licensing Information: You are free to use or extend these projects for 
 education or research purposes provided that (1) you retain this notice
 and (2) you provide clear attribution to UC Berkeley, including a link 
 to http://barc-project.com

 Attibution Information: The barc project ROS code-base was developed
 at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
 (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
 by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
 based on an open source project by Bruce Wootton
=# 

# This is an implementation of MPC using the kinematic bicycle model found here:
# http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf

__precompile__()

module GPSKinMPCPathFollowerDelay

    using JuMP
    using Ipopt

    #### (1) Initialize model and model parameters and MPC gains ####
	dt_control = 0.05		# control period, ts (s)
    mdl = Model(solver = IpoptSolver(print_level=0, max_cpu_time = dt_control))

	L_a     = 1.5213 		# dist from CoG to front axle (m)
	L_b     = 1.4987 		# dist from CoG to rear axle (m)
	dt      = 0.20			# model discretization time, td (s)
	N       = 8				# horizon
   
	v_ref = 1.0							    # target velocity for path follower
	x_ref   = v_ref*collect(0.0:dt:N*dt)	# target x(0.0 s), ..., x(N*dt s)
	y_ref   = zeros(N+1)					# target y(0.0 s), ..., y(N*dt s)
    psi_ref = zeros(N+1)					# target psi(0.0 s), ..., psi(N*dt s)

    steer_max = 0.5 		# tire angle (d_f) bound, rad
    steer_dmax = 0.5		# tire angle (d_f) rate bound, rad/s

	a_min = -3.0			# acceleration and deceleration bounds, m/s^2
	a_max = 2.0				
	a_dmax = 1.5			# jerk bound, m/s^3

	v_min = 0.0				# vel bounds (m/s)
	v_max = 20.0
	
	time_constant_acc = 0.4 # s
	alpha = dt_control/(time_constant_acc + dt_control) # LPF gain from https://en.wikipedia.org/wiki/Low-pass_filter
	
    # Cost function gains.
	@NLparameter(mdl, C_x    == 9.0) # longitudinal deviation
	@NLparameter(mdl, C_y    == 9.0) # lateral deviation
	@NLparameter(mdl, C_psi  == 10.0) # heading deviation
	@NLparameter(mdl, C_v    == 0.0) # target velocity deviation

	@NLparameter(mdl, C_dacc == 100.0) # jerk
	@NLparameter(mdl, C_ddf  == 1000.0) # slew rate
	@NLparameter(mdl, C_acc  == 0.0)   # acceleration
	@NLparameter(mdl, C_df   == 0.0)   # tire angle

	#### (2) Define State/Input Variables and Constraints ####
	# states: position (x,y), velocity (v), heading (psi)
	# inputs: tire angle (d_f) and acceleration (acc).
	#println("Creating kinematic bicycle model ....")
	@variable( mdl, x[1:(N+1)], start=0.0)
	@variable( mdl, y[1:(N+1)], start=0.0)
	#@variable( mdl, v_min <= v[1:(N+1)] <= v_max, start=0.0)
	@variable( mdl, v[1:(N+1)], start=0.0)
	@variable( mdl, psi[1:(N+1)], start=0.0)
	@variable( mdl, acc_lp[1:(N+1)], start=0.0) # low pass filtered acceleration.

	# Input Constraints
	@variable( mdl, a_min <= acc[1:N] <= a_max, start=0.0)
	@variable( mdl, -steer_max <= d_f[1:N] <= steer_max, start=0.0)

	# Input Steering Rate Constraints (with slack)
	@variable( mdl, sl_df[1:N] >= 0.0, start=0.0) # slack on steering
	@NLparameter(mdl, d_f_current == 0.0) # Current tire angle is a model parameter.
	#@NLconstraint(mdl, -steer_dmax*dt_control - sl_df[1] <= d_f[1]  - d_f_current <= steer_dmax*dt_control + sl_df[1])
	@NLconstraint(mdl, d_f[1]  - d_f_current + sl_df[1] >= -steer_dmax*dt_control)
	@NLconstraint(mdl, d_f[1]  - d_f_current - sl_df[1] <= steer_dmax*dt_control)
    for i in 1:(N-1)
        #@constraint(mdl, -steer_dmax*dt - sl_df[i+1] <= d_f[i+1] - d_f[i] <= steer_dmax*dt + sl_df[i+1])
		@constraint(mdl, d_f[i+1] - d_f[i] + sl_df[i+1] >= -steer_dmax*dt)
		@constraint(mdl, d_f[i+1] - d_f[i] - sl_df[i+1] <= steer_dmax*dt)
    end

	# Input Acceleration Rate Constraints (with slack)
	@variable( mdl, sl_acc[1:N] >= 0.0, start=0.0) # slack on acceleration
	@NLparameter(mdl, acc_current == 0.0) # Current acceleration is a model parameter.
	#@NLconstraint(mdl, -a_dmax*dt_control - sl_acc[1] <= acc[1]  - acc_current <= a_dmax*dt_control + sl_acc[1])
	@NLconstraint(mdl, acc[1]  - acc_current + sl_acc[1] >= -a_dmax*dt_control)
	@NLconstraint(mdl, acc[1]  - acc_current - sl_acc[1] <= a_dmax*dt_control)
    
	for i in 1:(N-1)
        #@constraint(mdl, -a_dmax*dt - sl_acc[i+1] <= acc[i+1] - acc[i] <= a_dmax*dt + sl_acc[i+1])
		@constraint(mdl, acc[i+1] - acc[i] + sl_acc[i+1] >= -a_dmax*dt)
		@constraint(mdl, acc[i+1] - acc[i] - sl_acc[i+1] <= a_dmax*dt)
    end	

	#### (3) Define Objective ####

	# Reference trajectory is encoded as model parameters.
	@NLparameter(mdl, x_r[i=1:(N+1)] == x_ref[i]) 
	@NLparameter(mdl, y_r[i=1:(N+1)] == y_ref[i])
	@NLparameter(mdl, psi_r[i=1:(N+1)] == psi_ref[i])
	@NLparameter(mdl, v_target == v_ref)

	# Cost function.
    @NLobjective(mdl, Min, sum{ C_x*(x[i] - x_r[i])^2 + C_y*(y[i] - y_r[i])^2 + C_psi*(psi[i] - psi_r[i])^2 , i=2:(N+1)} + 
                           C_v *sum{ (v[i] - v_target)^2, i = 2:N} + 
                           C_acc*sum{(acc[i])^2, i=1:N} +
                           C_df*sum{(d_f[i])^2, i=1:N} +
						   C_dacc*sum{(acc[i+1] - acc[i])^2, i=1:(N-1)} +
                           C_ddf*sum{(d_f[i+1] - d_f[i])^2, i=1:(N-1)} + 
						   sum{ sl_df[i] + sl_acc[i], i=1:N}
				)

	#### (4) Define System Dynamics Constraints ####
	# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
	#               Spring, 2011, page 26

    # Initial condition is a model parameter.
	@NLparameter(mdl, x0     == 0.0); @NLconstraint(mdl, x[1]     == x0);    
	@NLparameter(mdl, y0     == 0.0); @NLconstraint(mdl, y[1]     == y0);    
	@NLparameter(mdl, psi0   == 0.0); @NLconstraint(mdl, psi[1]   == psi0);
    @NLparameter(mdl, v0     == 0.0); @NLconstraint(mdl, v[1]     == v0);

	@NLexpression(mdl, bta[i = 1:N], atan( L_b / (L_a + L_b) * tan(d_f[i]) ) )

	@NLconstraint(mdl, acc_lp[1] == acc_current)
	for i in 1:N
        # equations of motion wrt CoG
		@NLconstraint(mdl, x[i+1]   == x[i]   + dt*( v[i]*cos(psi[i] + bta[i]) ) )
		@NLconstraint(mdl, y[i+1]   == y[i]   + dt*( v[i]*sin(psi[i] + bta[i]) ) )
		@NLconstraint(mdl, psi[i+1] == psi[i] + dt*( v[i]/L_b*sin(bta[i]) ) )
        @NLconstraint(mdl, v[i+1]   == v[i]   + dt*( acc_lp[i] ) )
        @NLconstraint(mdl, acc_lp[i+1] == (1.0-alpha) * acc_lp[i] + alpha * acc[i] )
	end

    #### (5) Initialize Solver ####
	#println("MPC: Initial solve ...")
	status = solve(mdl)
	#println("MPC: Finished initial solve: ", status)
	
	#################################
	##### State Update Function #####
    function update_init_cond(x::Float64, y::Float64, psi::Float64, vel::Float64)
        # update mpc initial condition 
        setvalue(x0,    x)
        setvalue(y0,    y)
        setvalue(psi0,  psi)
        setvalue(v0,    vel)
    end

	#####################################
	##### Reference Update Function #####
    function update_reference(x_ref::Array{Float64,1}, y_ref::Array{Float64,1}, psi_ref::Array{Float64,1}, v_ref::Float64)
    	# update the reference trajectory for MPC.
    	setvalue(x_r[i=1:(N+1)], x_ref) 
    	setvalue(y_r[i=1:(N+1)], y_ref) 
    	setvalue(psi_r[i=1:(N+1)], psi_ref)
    	setvalue(v_target, v_ref)
    end

	#########################################
	##### Input Update Function #####
	function update_current_input(c_swa::Float64, c_acc::Float64)
		setvalue(d_f_current, c_swa)
		setvalue(acc_current, c_acc)
	end

	#########################################
	##### Cost Update Function #####
	function update_cost(cx::Float64, cy::Float64, cp::Float64, cv::Float64,
						 cda::Float64, cdd::Float64, ca::Float64, cd::Float64)
		setvalue(C_x,   cx)
		setvalue(C_y,   cy) 
		setvalue(C_psi, cp)
		setvalue(C_v,   cv)

		setvalue(C_dacc, cda)
		setvalue(C_ddf,  cdd)
		setvalue(C_acc,  ca)
		setvalue(C_df,   cd)
	end

	#################################
	##### Model Solve Function #####
    function solve_model()
        # Solve the model, assuming relevant update functions have been called by the user.
        tic()
        status = solve(mdl)
        solv_time = toq()

        # get optimal solutions
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])
        return acc_opt[1], d_f_opt[1], status, solv_time
    end

	#################################
	##### Diagnostics Function ######
	function get_solver_results()
		# This function gets all relevant solver variables and returns it to the client.
		# Handy for debugging or logging full results.

		# State Variables and Reference
		x_mpc   = getvalue(x[1:(N+1)])
		y_mpc   = getvalue(y[1:(N+1)])
		v_mpc   = getvalue(v[1:(N+1)])
		psi_mpc = getvalue(psi[1:(N+1)])

		x_ref   = getvalue(x_r[1:(N+1)])
		y_ref   = getvalue(y_r[1:(N+1)])
		psi_ref = getvalue(psi_r[1:(N+1)])
		v_ref   = getvalue(v_target)

		# Optimal Solution
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])

		return x_mpc, y_mpc, v_mpc, psi_mpc, x_ref, y_ref, v_ref, psi_ref, d_f_opt, acc_opt	
	end
end
