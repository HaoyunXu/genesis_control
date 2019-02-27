import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from genesis_msgs.msg import ESRTrackReport
from genesis_msgs.msg import MandoObjectReport
import rospy
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import math

img = None
radar = [dict() for i in range(64)]
objects = [dict() for i in range(8)]

def parseImg(msg):
	global img
	try:
		img = CvBridge().imgmsg_to_cv2(msg, "rgb8")
	except CvBridgeError as e:
		print e

def parseRadar(msg):
	global radar

	index  = int(msg.header.frame_id) - 1
	status = msg.status # > 0 = valid target
	mode   = msg.mode   # > 0 = detected by either MR/LR
	dist   = msg.range
	vel    = msg.range_rate
	acc    = msg.range_accel
	angle  = msg.angle
	width = msg.width # m
	rolling_count = msg.rolling_count

	radar[index]['status'] = status  # New_Coasted_Target(7), Invalid_Coasted_Target(6), Merged_Target(5), Coasted_Target(4),
                                         # Updated_Target(3), New_Updated_Target(2), New_Target(1), No_Target(0)
	radar[index]['mode'] = mode
	radar[index]['dist'] = dist
	radar[index]['vel'] = vel   # m/s
	radar[index]['acc'] = acc
	radar[index]['angle'] = angle
	radar[index]['width'] = width
	radar[index]['rolling_count'] = rolling_count

def parseObject(msg):
	global objects

	index  = msg.index - 1
	obj_id  = msg.object_identifier # > 0 = a target
	valid   = msg.object_valid
	x      = msg.range
	y      = msg.position_y
	range_rate = msg.range_rate
	motion_status = msg.motion_status
	object_age = msg.object_age
	object_lane = msg.object_lane

	objects[index]['obj_id'] = obj_id
	objects[index]['valid'] = valid
	objects[index]['x'] = x
	objects[index]['y'] = y
	objects[index]['range_rate'] = range_rate       # m/s
	objects[index]['motion_status'] = motion_status # stationary (4), stopped(3), oncoming (2), preceding(1), undecided(0)
	objects[index]['object_age'] = object_age
	objects[index]['object_lane'] = object_lane


def radar_draw_loop():
	rospy.init_node('radar_viz', anonymous = True)
	rospy.Subscriber('/image_raw', Image, parseImg, queue_size = 2)
	rospy.Subscriber('/mando_radar/esr_track', ESRTrackReport, parseRadar, queue_size = 2)
	rospy.Subscriber('/mando_camera/object_detection', MandoObjectReport, parseObject, queue_size = 2)
	r = rospy.Rate(10.0)

	f, (ax1,ax2) = plt.subplots(2)

	plt.ion()


	global img, radar
	while not rospy.is_shutdown():

		if img is None:
			print('Waiting: img')
			r.sleep()
			continue
		img = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
		ax1.imshow(img)


		ax2.clear()

		for i in range(64):
			if 'status' in radar[i].keys() and radar[i]['status'] > 0:

				width = radar[i]['width']       # Not very accurate. Car: usually 1 meter. Pedestrians: 0 meter
				velocity = radar[i]['vel']      # m/s
				distance = radar[i]['dist']
				rolling_count = radar[i]['rolling_count']


				heading  = -math.radians(radar[i]['angle']) + math.pi/2

				obj_x = distance * math.cos(heading)
				obj_y = distance * math.sin(heading)
				velocity_kmph = velocity * 3.6  # km/h

				#print 'status = ', radar[i]['status']  # A lot of 3 (updated targets) and 4 (coasted targets) What does it mean ?

				#Keep targets in front of the car
				if abs(obj_x) < 6 and abs(obj_y) < 40:
					ax2.plot(obj_x, obj_y, 'rx', markersize=8)
					#print width
					#print 'Velocity_kmph = ', velocity_kmph    # When comparing with wheel speed, they are approximatively the same.
										   # Try to plot both speeds and compare them
										   # -> Try to substract both to get relative speed

				# --------- Basic Velocity Filter ---------------
				#if velocity > 0.1:
				#	ax1.plot(obj_x, obj_y, 'rx', markersize=8)
				#	print velocity
				# ------------------------

		for i in range(8):
			if 'obj_id' in objects[i].keys() and objects[i]['obj_id'] > 0:

				x = objects[i]['x']
				y = objects[i]['y']
				valid = objects[i]['valid']
				motion_status = objects[i]['motion_status']
				range_rate = objects[i]['range_rate']
				object_age = objects[i]['object_age']        # Pretty accurate. Unit ? [not time, probably instance]. max = 254
				object_lane = objects[i]['object_lane']


				# x and y are permuted, this is why we have plot(y,x)
				if valid > 0 and object_age > 0:
					ax2.plot(-y, x, 'go', markersize=8)
					#print 'motion status = ', motion_status
					print 'age = ', object_age
					#print 'range rate = ', range_rate, 'm/s'
					#print '------'
					#plt.text(-y + 0.5, x + 0.5, 'Object valid ', fontsize=10) #Just an example
					plt.text(-y + 4, x + 0.5, range_rate, fontsize=10)
					#plt.text(-y + 4, x + 0.5, 'age = ', fontsize=10)
					#plt.text(-y + 8, x + 0.9, object_age, fontsize=10)


				else:
			 		ax2.plot(-y, x, 'ro', markersize=8)
				 	print 'Not valid'
				 	#print 'range rate = ', range_rate
					plt.text(-y + 0.5, x + 0.5, 'Object not valid', fontsize=10)
					print object_age
				 	print '------'

		plt.xlim([-20, 20])
		plt.ylim([-1, 40])
		plt.grid()

		f.canvas.draw()

		plt.pause(0.001)
		r.sleep()

if __name__=='__main__':
	radar_draw_loop()
