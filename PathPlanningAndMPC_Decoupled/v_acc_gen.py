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

received = False
v_ref = None
human_take_over = False

def v_acc_callback(msg):
	global received, v_ref, human_take_over
	received = True
	#unpack
	vf = msg.data[0]   # front car vel
	ve = msg.data[1]   # self vel
	d = msg.data[2]    # relative distance

	N = 17
	v_ref = ve*np.ones(N)
	deacc_max = -2.0
	# The max deacceleration
	d_brake = abs((ve**2-vf**2)/(2*deacc_max)) # The braking distance for max deacceleration
	d_safe = d_brake + abs(ve-vf)*1   # braking distance + buffer (for one second reaction)

	if (vf < ve):
		v_ref[0] = ve
		v_ref[N-1] = vf

		if d >= d_safe:

			for i in range(1,N-1):
				v_ref[i] = v_ref[i-1]-(ve-vf)/(N-2)  # Do linear deacceleration to be the front car velocity

		else: # Send message to driver take-over
			human_take_over = True



def pub_loop(v_acc_pub_obj, human_take_over_pub_obj):
	global received, v_ref, human_take_over
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		if human_take_over:
			human_take_over_pub_obj.publish(BoolMsg(data=True))
			human_take_over = False
		elif received:
			v_acc_pub_obj.publish(Float32MultiArray(data=v_ref))
		else:
			v_acc_pub_obj.publish(Float32MultiArray(data=None))
		received = False
		rate.sleep()


def start_node():
	rospy.init_node('v_acc_node')
	rospy.Subscriber('acc_state',Float32MultiArray, v_acc_callback, queue_size=2)
	v_acc_pub = rospy.Publisher("v_acc", Float32MultiArray, queue_size=2)
	human_take_over_pub = rospy.Publisher("takeover", BoolMsg, queue_size=2)
	pub_loop(v_acc_pub,human_take_over_pub)


if __name__=='__main__':
	try:
		start_node()
	except rospy.ROSInterruptException:
		pass
