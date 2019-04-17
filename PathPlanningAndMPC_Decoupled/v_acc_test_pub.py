#!/usr/bin/env python
import numpy as np
import math
import rospy
from std_msgs.msg import Float32 as Float32Msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool as BoolMsg
from genesis_control.msg import state_est
from genesis_control.msg import Multi_targets
from genesis_control.msg import target

def state_est_callback(msg):
	global pub
	x_curr = msg.x
	y_curr = msg.y
	psi_curr = msg.psi
	v_curr = msg.v

	#test case 1: Assume there is a stationary car
	x_f = -133.409
	y_f = -368.668
	v_f = 0

	d_relative = math.sqrt((x_f - x_curr)**2 + (y_f - y_curr)**2)


	acc = Multi_targets()
	target_avg = target()
	target_avg.pos_x    = 0  # not used
	target_avg.pos_y    = d_relative
	target_avg.speed    = v_f-v_curr
	target_avg.category = 0  # not used
	target_avg.counter  = 0  # not used
	acc.data.append(target_avg)

	pub.publish(acc)


def start_node():
	global pub
	rospy.init_node('v_acc_test_node')
	pub = rospy.Publisher('/vehicle/radar_targets_acc', Multi_targets, queue_size=2)
	rospy.Subscriber("/vehicle/state_est", state_est, state_est_callback, queue_size=2)
	rospy.spin()


if __name__ == '__main__':
  try:
    start_node()
  except rospy.ROSInterruptException:
    pass
