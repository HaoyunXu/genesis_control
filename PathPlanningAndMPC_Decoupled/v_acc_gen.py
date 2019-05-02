#!/usr/bin/env python
# This function generates the velocity profile for MPC. The velocity profile is an
# array of velocity values, which illustrates the change of velocity from current
# one to that of front car. The size of array is the horizon of MPC.

# The inputs of this function are vf (the velocity of front car), ve (the velocity
# of ego car), and d (the distance between two cars). It will return velocity
# profile only in safe conditions (distance is greater than safe distance, and
# deacceleration is in the max deacceleration boundary). Otherwise it will publish
# a True boolean message as "takeover"

import numpy as np
import rospy
from std_msgs.msg import Float32 as Float32Msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool as BoolMsg
from genesis_control.msg import target
from genesis_control.msg import Multi_targets
from genesis_control.msg import state_est

valid = False
v_ref = None
human_take_over = False


# 1: published v_ref in previous iteration
# 2: did not publish
prev_state = 2

def state_est_callback(msg):
	global ve
	ve = msg.v


def v_acc_callback(msg):
	if not msg.data:
		return

	global v_acc_pub, human_take_over_pub,ve, prev_state


	#unpack
	v_relative = msg.data[0].speed
	vf = v_relative + ve   # front car vel

	#in case the car is driving backward
	# it's either the car is actaully driving backward, or the output of radar+vision is inaccurate
	if vf<-0.2:
		human_take_over = True
		return

	d = msg.data[0].pos_y         # relative distance



	# if vf < ve
	#	if outside safety distance
	#		calculate braking
	#		if braking is significant
	#			calculate v_ref, enable acc
	#		else: when braking is small
	#			brake later, acc not enabled for now
	# 	else: within safety distance
	#		human take over
	# if vf > ve acc not enabled

	buffer = 0.3

	if abs(v_relative)<0.1 and d<10:
		v_ref = [ve]*17
		v_acc_pub.publish(Float32MultiArray(data=v_ref))


	elif (vf < ve):
		N = 17
		v_ref = ve*np.ones(N)
		v_ref[0] = ve



		dt = 0.1 # dt for mpc

		deacc_max = -2.0
		# The max deacceleration
		d_brake = abs((ve**2-vf**2)/(2*deacc_max)) # The braking distance for max deacceleration
		d_safe = d_brake + abs(ve-vf)*1 + 3   # braking distance + buffer (for one second reaction) + even stationary we want 3m distance


		if d >= d_safe:
			distance_to_brake = d-d_safe
			time = distance_to_brake/(v_relative/2+ve)
			acc_braking = v_relative/time
			if (prev_state==1 and acc_braking < -0.65+buffer) or (prev_state!=1 and acc_braking < -0.65): #if braking is significant, brake
				for i in range(1,N):
					res = v_ref[i-1] + dt*acc_braking  # Do linear deacceleration to be the front car velocity
					if (res>0):
						v_ref[i] = res
					else:
						v_ref[i] = 0
				v_acc_pub.publish(Float32MultiArray(data=v_ref))
				prev_state = 1

			elif d<=d_safe*2:
				v_ref = [ve]*17
				v_acc_pub.publish(Float32MultiArray(data=v_ref))

			else:  #if braking is too small, brake later
				v_acc_pub.publish(Float32MultiArray(data=None))
				prev_state = 2

		elif d<d_safe: # if inside braking distance, send message to driver take-over
			human_take_over_pub.publish(BoolMsg(data=True))
	else: # when vf > ve, don't do anything
		v_acc_pub.publish(Float32MultiArray(data=None))


# def pub_loop(v_acc_pub_obj, human_take_over_pub_obj):
# 	global valid, v_ref, human_take_over
# 	rate = rospy.Rate(20)
# 	while not rospy.is_shutdown():
# 		if human_take_over:
# 			human_take_over_pub_obj.publish(BoolMsg(data=True))
# 			human_take_over = False
# 		elif valid:
# 			v_acc_pub_obj.publish(Float32MultiArray(data=v_ref))
# 		else:
# 			v_acc_pub_obj.publish(Float32MultiArray(data=None))
# 		valid = False
# 		rate.sleep()


def start_node():
	global v_acc_pub, human_take_over_pub
	rospy.init_node('v_acc_node')
	rospy.Subscriber('radar_targets_acc',Multi_targets, v_acc_callback, queue_size=2)
	rospy.Subscriber('state_est', state_est, state_est_callback, queue_size=2)
	v_acc_pub = rospy.Publisher("v_acc", Float32MultiArray, queue_size=2)
	human_take_over_pub = rospy.Publisher("takeover", BoolMsg, queue_size=2)
	rospy.spin()



if __name__=='__main__':
	try:
		start_node()
	except rospy.ROSInterruptException:
		pass
