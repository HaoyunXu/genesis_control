import matplotlib.pyplot as plt
import rospy
import numpy as np
from genesis_control.msg import state_est
from genesis_control.msg import Multi_targets
from genesis_control.msg import target

d_safe = []
d_brake = []
dist = []
ve = []
vf = []
d_safe_l = []
d_brake_l = []
dist_l = []
ve_l = []
vf_l = []

def state_est_callback(msg):
	global ve
	if not ve:
		ve = [msg.v]
	else:
		ve = ve + [msg.v]

def v_acc_callback(msg):
	global dist, d_brake, d_safe, ve, vf, counter
	#unpack
	v_relative = msg.data[0].speed
	deacc_max = -2.0
	if not vf:
		vf = [v_relative]
		dist = [msg.data[0].pos_y]
		d_brake = [abs((ve[-1]**2-vf[-1]**2)/(2*deacc_max))]
		d_safe = [d_brake[-1] + abs(ve[-1]-vf[-1])*1 + 3]
	else:
		vf = vf + [v_relative + ve[-1]]
		dist = dist + [msg.data[0].pos_y]         # relative distance
		d_brake = d_brake + [abs((ve[-1]**2-vf[-1]**2)/(2*deacc_max))] # The braking distance for max deacceleration
		d_safe = d_safe + [d_brake[-1] + abs(ve[-1]-vf[-1])*1 + 3]   # braking distance + buffer (for one second reaction) + even stationary we want 3m distance

	# dist = dist + [msg.data[0].pos_y]         # relative distance
	# d_brake = d_brake + [abs((ve**2-vf**2)/(2*deacc_max))] # The braking distance for max deacceleration
	# d_safe = d_safe + [d_brake[-1] + abs(ve-vf)*1 + 3]   # braking distance + buffer (for one second reaction) + even stationary we want 3m distance

def main():
	global dist, d_brake, d_safe, ve, vf, dist_l, ve_l, vf_l, d_safe_l, d_brake_l
	t = [0]
	print(t)
	print(dist)
	while not rospy.is_shutdown():
		# print(t)
		# print(dist)
		# plt.show()
		if len(dist) != 0:
			d_safe_l = d_safe_l+[d_safe[-1]]
			d_brake_l = d_brake_l+[d_brake[-1]]
			dist_l = dist_l+[dist[-1]]
			ve_l = ve_l+[ve[-1]]
			vf_l = vf_l+[vf[-1]]
			plt.figure(1)
			plt.plot(t, dist_l,'r',label='Distance from Front Vehicle' if len(t)==1 else '')
			plt.plot(t,d_brake_l,'b',label='Braking Distance' if len(t)==1 else '')
			plt.plot(t,d_safe_l,'g',label='Safety Distance'if len(t)==1 else '')
			plt.legend(loc='upper left')
			plt.xlabel('Time')
			plt.ylabel('Distance (m)')
			plt.pause(0.01)
			# plt.draw()
			plt.figure(2)
			plt.plot(t, ve_l,'m',label='Velocity Ego' if len(t)==1 else '')
			plt.plot(t,vf_l,'c',label='Front Car Velocity' if len(t)==1 else '')
			plt.legend(loc='upper left')
			plt.xlabel('Time')
			plt.ylabel('Velocity (m/s)')
			plt.pause(0.01)
			# plt.draw()
			t=t+[t[-1]+1]
		# first plot: d vs d_brake vs d_safe
		# second plot: ve vs vf
	plt.legend()
	plt.show()
def start_node():
	global v_acc_pub, human_take_over_pub
	rospy.init_node('acc_plotter_node')
	rospy.Subscriber('vehicle/radar_targets_acc',Multi_targets, v_acc_callback, queue_size=2)
	rospy.Subscriber('vehicle/state_est', state_est, state_est_callback, queue_size=2)
	main()

if __name__=='__main__':
	try:
		start_node()
	except rospy.ROSInterruptException:
		pass
