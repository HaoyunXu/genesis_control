#!/usr/bin/env julia

# MPC Command Publisher/Controller Module Interface to the Genesis.
# This version using the Nonlinear Kinematic Bicycle Model.

###########################################
#### ROBOTOS
###########################################
using RobotOS
@rosimport genesis_path_follower.msg: state_est
@rosimport genesis_path_follower.msg: mpc_path
@rosimport std_msgs.msg: UInt8
@rosimport std_msgs.msg: Float32
rostypegen()
using genesis_path_follower.msg
using std_msgs.msg
using PyCall

###########################################
#### LOAD ROSPARAMS
###########################################
if has_param("mat_waypoints")
	mat_fname = get_param("mat_waypoints")
else
	error("No Matfile of waypoints provided!")
end

if has_param("track_using_time") && has_param("target_vel")
	track_with_time = get_param("track_using_time")
	target_vel = get_param("target_vel")	
else
	error("Invalid rosparam trajectory definition: track_using_time and target_vel")
end

if has_param("scripts_dir")
	scripts_dir = get_param("scripts_dir")
else
	error("Did not provide the scripts directory!")
end

if(has_param("lat0") && has_param("lat0") && has_param("lat0"))
	lat0 = get_param("lat0")
	lon0 = get_param("lon0")
	yaw0 = get_param("yaw0")
else
	error("Invalid rosparam global origin provided!")
end

###########################################
#### Reference GPS Trajectory Module
###########################################
# Access Python modules for path processing.  Ugly way of doing it, can seek to clean this up in the future.
using PyCall
const gps_utils_loc = scripts_dir * "gps_utils"
unshift!(PyVector(pyimport("sys")["path"]), gps_utils_loc) # append the current directory to Python path
@pyimport ref_gps_traj as rgt
grt = rgt.GPSRefTrajectory(mat_filename=mat_fname, LAT0=lat0, LON0=lon0, YAW0=yaw0)

###########################################
#### MPC Controller Module with Cost Function Weights.
#### Global Variables for Callbacks/Control Loop.
###########################################
push!(LOAD_PATH, scripts_dir * "mpc_utils")
import GPSKinMPCPathFollower
const kmpc = GPSKinMPCPathFollower
kmpc.update_cost(9.0, 9.0, 10.0, 0.0, 100.0, 1000.0, 0.0, 0.0) # x,y,psi,v,da,ddf,a,df

# Reference for MPC
const t_ref = collect(0:kmpc.dt:kmpc.N*kmpc.dt)
x_ref = zeros(length(t_ref))
y_ref = zeros(length(t_ref))
psi_ref = zeros(length(t_ref))

if target_vel > 0.0
	des_speed = target_vel
else
	des_speed = 0.00
end

ref_lock = false				
received_reference = false
x_curr  = 0.0
y_curr  = 0.0
psi_curr  = 0.0
v_curr  = 0.0
command_stop = false

###########################################
#### State Estimation Callback.
###########################################
function state_est_callback(msg::state_est)

	global x_curr, y_curr, psi_curr, v_curr
	global received_reference

	if ref_lock == false
		x_curr = msg.x
		y_curr = msg.y
		psi_curr = msg.psi
		v_curr = msg.v
		received_reference = true
	end
end

function pub_loop(acc_pub_obj, steer_pub_obj, mpc_path_pub_obj)
    loop_rate = Rate(20.0)
    while ! is_shutdown()
	    if ! received_reference		# Reference not received so don't use MPC yet.
	        rossleep(loop_rate)
	        continue
	    end

	    global ref_lock				# Ref lock used to ensure that get/set of state doesn't happen simultaneously.
	    ref_lock = true

		global x_curr, y_curr, psi_curr, v_curr, des_speed, command_stop

		if ! track_with_time		
			# fixed velocity-based path tracking
			x_ref, y_ref, psi_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr, des_speed)

			if stop_cmd == true
				command_stop = true
			end

		else
			# trajectory tracking
			x_ref, y_ref, psi_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr)

			if stop_cmd == true
				command_stop = true
			end
		end
		
		# Update Model
		kmpc.update_init_cond(x_curr, y_curr, psi_curr, v_curr)
		kmpc.update_reference(x_ref, y_ref, psi_ref, des_speed)

	    ref_lock = false
		
		if command_stop == false
			a_opt, df_opt, is_opt, solv_time = kmpc.solve_model()

			rostm = get_rostime()
			tm_secs = rostm.secs + 1e-9 * rostm.nsecs

		    log_str = @sprintf("Solve Status: %s, Acc: %.3f, SA: %.3f, ST: %.3f", is_opt, a_opt, df_opt, solv_time)
		    loginfo(log_str)

			publish( acc_pub_obj,   Float32Msg(a_opt) )
			publish( steer_pub_obj, Float32Msg(df_opt) )

			kmpc.update_current_input(df_opt, a_opt)
			res = kmpc.get_solver_results()

			mpc_path_msg = mpc_path()
			mpc_path_msg.header.stamp = rostm
			mpc_path_msg.solv_status  = string(is_opt)
			mpc_path_msg.solv_time = solv_time
			mpc_path_msg.xs   = res[1] 	# x_mpc
			mpc_path_msg.ys   = res[2] 	# y_mpc
			mpc_path_msg.vs   = res[3]	# v_mpc
			mpc_path_msg.psis = res[4] 	# psi_mpc	
			mpc_path_msg.xr   = res[5] 	# x_ref
			mpc_path_msg.yr   = res[6] 	# y_ref
			mpc_path_msg.vr   = [res[7]]	# v_ref
			mpc_path_msg.psir = res[8] 	# psi_ref
			mpc_path_msg.df   = res[9]	# d_f
			mpc_path_msg.acc  = res[10]	# acc
			publish(mpc_path_pub_obj, mpc_path_msg)
		else
			publish( acc_pub_obj,   Float32Msg(-1.0) )
			publish( steer_pub_obj, Float32Msg(0.0) )		
		end
		
	    rossleep(loop_rate)
	end
end	

function start_mpc_node()
    init_node("dbw_mpc_pf")
    acc_pub   = Publisher("/control/accel", Float32Msg, queue_size=2)
    steer_pub = Publisher("/control/steer_angle", Float32Msg, queue_size=2)

    acc_enable_pub   = Publisher("/control/enable_accel", UInt8Msg, queue_size=2, latch=true)
    steer_enable_pub = Publisher("/control/enable_spas",  UInt8Msg, queue_size=2, latch=true)

    mpc_path_pub = Publisher("mpc_path", mpc_path, queue_size=2)
	sub_state  = Subscriber("state_est", state_est, state_est_callback, queue_size=2)

	# Start up Ipopt/Solver.
	for i=1:3
		kmpc.solve_model()
	end    

	publish(acc_enable_pub, UInt8Msg(2))
	publish(steer_enable_pub, UInt8Msg(1))

    pub_loop(acc_pub, steer_pub, mpc_path_pub)    
end

if ! isinteractive()
	try 
    	start_mpc_node()
    catch x
    	print(x)
    end
end
