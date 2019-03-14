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

# radar = [dict() for i in range(64)]
# objects = [dict() for i in range(8)]

radar = dict()
radar = {}
objects = dict()
objects = {}

# def parseImg(msg):
# 	global img
# 	try:
# 		img = CvBridge().imgmsg_to_cv2(msg, "rgb8")
# 	except CvBridgeError as e:
# 		print e

def parseRadar(msg):
	global radar

	# index  = int(msg.header.frame_id) - 1
	status = msg.status # > 0 = valid target
	mode   = msg.mode   # > 0 = detected by either MR/LR
	dist   = msg.range
	vel    = msg.range_rate
	acc    = msg.range_accel
	angle  = msg.angle
	width  = msg.width # m
	rolling_count = msg.rolling_count


	radar['status']  = status  # New_Coasted_Target(7), Invalid_Coasted_Target(6), Merged_Target(5), Coasted_Target(4),
                                         # Updated_Target(3), New_Updated_Target(2), New_Target(1), No_Target(0)
	radar['mode']    = mode
	radar['dist']    = dist
	radar['vel']     = vel   # m/s
	radar['acc']     = acc
	radar['angle']   = angle
	radar['width']   = width
	radar['rolling_count'] = rolling_count

# def parseObject(msg):
# 	global objects
#
# 	index  = msg.index - 1
# 	obj_id  = msg.object_identifier # > 0 = a target
# 	valid   = msg.object_valid
# 	x      = msg.range
# 	y      = msg.position_y
# 	range_rate = msg.range_rate
# 	motion_status = msg.motion_status
# 	object_age = msg.object_age
# 	object_lane = msg.object_lane
#
# 	objects['obj_id'] = obj_id
# 	objects['valid'] = valid
# 	objects['x'] = x
# 	objects['y'] = y
# 	objects['range_rate'] = range_rate       # m/s
# 	objects['motion_status'] = motion_status # stationary (4), stopped(3), oncoming (2), preceding(1), undecided(0)
# 	objects['object_age'] = object_age
# 	objects['object_lane'] = object_lane


def radar_draw_loop():
	rospy.init_node('radar_viz', anonymous = True)
	#rospy.Subscriber('/image_raw', Image, parseImg, queue_size = 2)
	rospy.Subscriber('/mando_radar/esr_track', ESRTrackReport, parseRadar, queue_size = 2)
	#rospy.Subscriber('/mando_camera/object_detection', MandoObjectReport, parseObject, queue_size = 2)
	r = rospy.Rate(10.0)

	# f, (ax1,ax2) = plt.subplots(2)
	#
	# plt.ion()


	# global img, radar
	global radar
	while not rospy.is_shutdown():
		print 'mode = '
		print ' '
		print radar.get('dist')

		# if img is None:
		# 	print('Waiting: img')
		# 	r.sleep()
		# 	continue
		# img = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
		# ax1.imshow(img)


		# ax2.clear()
		#
		# for i in range(64):
		# 	if radar[i]['status'] > 0:
		#
		# 		width = radar[i]['width']       # Not very accurate. Car: usually 1 meter. Pedestrians: 0 meter
		# 		velocity = radar[i]['vel']      # m/s
		# 		distance = radar[i]['dist']
		# 		rolling_count = radar[i]['rolling_count']
		#
		#
		# 		heading  = -math.radians(radar[i]['angle']) + math.pi/2
		#
		# 		obj_x = distance * math.cos(heading)
		# 		obj_y = distance * math.sin(heading)
		# 		velocity_kmph = velocity * 3.6  # km/h
		#
		#
		# 		if abs(obj_x) < 6 and abs(obj_y) < 40:
		# 			ax2.plot(obj_x, obj_y, 'rx', markersize=8)
		#
		#
		# for i in range(8):
		# 	if objects[i]['obj_id'] > 0:
		#
		# 		x = objects[i]['x']
		# 		y = objects[i]['y']
		# 		valid = objects[i]['valid']
		# 		motion_status = objects[i]['motion_status']
		# 		range_rate = objects[i]['range_rate']
		# 		object_age = objects[i]['object_age']        # Pretty accurate. Unit ? [not time, probably instance]. max = 254
		# 		object_lane = objects[i]['object_lane']
		#
		#
		# 		# x and y are permuted, this is why we have plot(y,x)
		# 		if valid > 0 and object_age > 0:
		# 			ax2.plot(-y, x, 'go', markersize=8)
		#
		#
		# 		else:
		# 	 		ax2.plot(-y, x, 'ro', markersize=8)


		r.sleep()

if __name__=='__main__':
	radar_draw_loop()
