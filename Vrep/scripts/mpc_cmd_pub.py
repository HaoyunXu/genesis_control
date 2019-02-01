#!/usr/bin/env python

# MPC Command Publisher/Controller Module Interface to the Genesis.
# This version using the Nonlinear Kinematic Bicycle Model.

###########################################
#### ROBOTOS
###########################################
import rospy
# from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import mpc_path
from std_msgs.msg import UInt8 as UInt8Msg
from std_msgs.msg import Float32 as Float32Msg
from sensor_msgs.msg import JointState as JointStateMsg
import numpy as np

###########################################
#### LOAD ROSPARAMS
###########################################
if rospy.has_param("mat_waypoints"):
	mat_fname = rospy.get_param("mat_waypoints")
else:
	raise ValueError("No Matfile of waypoints provided!")
###########################################

import ref_gps_traj as rgt
grt = rgt.GPSRefTrajectory(mat_filename=mat_fname)

###########################################
#### MPC Controller Module with Cost Function Weights.
#### Global Variables for Callbacks/Control Loop.
###########################################
import julia
jl = julia.Julia()
scripts_dir = rospy.get_param("scripts_dir")
add_path_script = 'push!(LOAD_PATH, "%s/")' % scripts_dir
jl.eval(add_path_script)
jl.eval('import GPSKinMPCPathFollower')
from julia import GPSKinMPCPathFollower as kmpc
kmpc.update_cost(9.0, 9.0, 10.0, 0.0, 100.0, 1000.0, 0.0, 0.0) # x,y,psi,v,da,ddf,a,df

# Reference for MPC
des_speed = 20.00

ref_lock = False
received_reference = False
x_curr  = 0.0
y_curr  = 0.0
psi_curr  = 0.0
v_curr  = 0.0
command_stop = False

###########################################
#### State Estimation Callback.
###########################################
def state_est_callback(msg):
	global x_curr, y_curr, psi_curr, v_curr
	global received_reference

	if ref_lock == False:
		x_curr = msg.position[0]
		y_curr = msg.position[1]
		psi_curr = msg.effort[0]
		v_curr = np.sqrt((msg.velocity[0])**2+(msg.velocity[1])**2)
		received_reference = True

def pub_loop(acc_pub_obj, steer_pub_obj):
	loop_rate = rospy.Rate(20.0)
	while not rospy.is_shutdown():
		if not received_reference:
			# Reference not received so don't use MPC yet.
			loop_rate.sleep()
			continue

		# Ref lock used to ensure that get/set of state doesn't happen simultaneously.
		global ref_lock
		ref_lock = True

		global x_curr, y_curr, psi_curr, v_curr, des_speed, command_stop

		#Get waypoints
		x_ref, y_ref, psi_ref, stop_cmd = grt.get_waypoints(x_curr, y_curr, psi_curr)
		if stop_cmd == True:
			command_stop = True

		print('x:',x_ref)
		print('y:',y_ref)
		print('psi:',psi_ref)
		print('v_curr',v_curr)
		print('des_speed',des_speed)


		# Update Model
		kmpc.update_init_cond(x_curr, y_curr, psi_curr, v_curr)
		kmpc.update_reference(x_ref, y_ref, psi_ref, des_speed)

		ref_lock = False

		if command_stop == False:
			a_opt, df_opt, is_opt, solv_time = kmpc.solve_model()

			rostm = rospy.get_rostime()
			tm_secs = rostm.secs + 1e-9 * rostm.nsecs

			log_str = "Solve Status: %s, Acc: %.3f, SA: %.3f, ST: %.3f" % (is_opt, a_opt, df_opt, solv_time)
			rospy.loginfo(log_str)

			if is_opt == 'Optimal':
				print('Its optimal')
				print('acc=',a_opt,'steer=',df_opt)
				acc_pub_obj.publish(Float32Msg(a_opt))
				steer_pub_obj.publish(Float32Msg(df_opt))

			kmpc.update_current_input(df_opt, a_opt)
			# res = kmpc.get_solver_results()

		else:
			acc_pub_obj.publish(Float32Msg(-1.0))
			steer_pub_obj.publish(Float32Msg(0.0))

		loop_rate.sleep()

def start_mpc_node():
	rospy.init_node("dbw_mpc_pf")
	acc_pub   = rospy.Publisher("/acc", Float32Msg, queue_size=2)
	steer_pub = rospy.Publisher("/steer", Float32Msg, queue_size=2)

	acc_enable_pub   = rospy.Publisher("/control/enable_accel", UInt8Msg, queue_size=2, latch=True)
	steer_enable_pub = rospy.Publisher("/control/enable_spas",  UInt8Msg, queue_size=2, latch=True)

	sub_state  = rospy.Subscriber("/carState", JointStateMsg, state_est_callback, queue_size=2)

	# Start up Ipopt/Solver.
	for i in range(3):
		kmpc.solve_model()

	acc_enable_pub.publish(UInt8Msg(2))
	steer_enable_pub.publish(UInt8Msg(1))
	pub_loop(acc_pub, steer_pub)

if __name__=='__main__':
	start_mpc_node()
