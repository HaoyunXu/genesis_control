import cv2
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from genesis_msgs.msg import ESRTrackReport
from cv_bridge import CvBridge, CvBridgeError
from genesis_msgs.msg import MandoObjectReport



img = None

# radar = [dict() for i in range(64)]
# objects = [dict() for i in range(8)]

radar = dict()
radar = {}

objects = dict()
objects = {}

def parseImg(msg):
 	global img
 	try:
 		img = CvBridge().imgmsg_to_cv2(msg, "rgb8")
 	except CvBridgeError as e:
 		print e

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

def parseObject(msg):
	global objects
	
 	#index   = msg.index - 1
 	obj_id        = msg.object_identifier # > 0 = a target
 	valid         = msg.object_valid
 	x             = msg.range
 	y             = msg.position_y
 	range_rate    = msg.range_rate
 	motion_status = msg.motion_status
 	object_age    = msg.object_age
 	object_lane   = msg.object_lane

 	objects['obj_id']        = obj_id
 	objects['valid']         = valid
 	objects['x']             = x
 	objects['y']             = y
 	objects['range_rate']    = range_rate       # m/s
 	objects['motion_status'] = motion_status # stationary (4), stopped(3), oncoming (2), preceding(1), undecided(0)
 	objects['object_age']    = object_age
 	objects['object_lane']   = object_lane


def radar_draw_loop():
	rospy.init_node('radar_viz', anonymous = True)
	rospy.Subscriber('/image_raw', Image, parseImg, queue_size = 2)
	rospy.Subscriber('/mando_radar/esr_track', ESRTrackReport, parseRadar, queue_size = 2)
	rospy.Subscriber('/mando_camera/object_detection', MandoObjectReport, parseObject, queue_size = 2)
	r = rospy.Rate(10.0)


# ---------------------------------------------------------
	f, (ax1,ax2) = plt.subplots(2)
	plt.ion()
# ---------------------------------------------------------

	global img, radar, objects

	while not rospy.is_shutdown():
		

		if img is None:
			print('Waiting: img')
			r.sleep()
			continue

		img = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
		ax1.imshow(img)

		ax2.clear()

		
# ------------------------------------------------------

		
		if radar != {}:

			radar_status = radar.get('status')
			radar_mode   = radar.get('mode')
			radar_dist   = radar.get('dist')
			radar_vel    = radar.get('vel')
			radar_acc    = radar.get('acc')
			radar_angle  = radar.get('angle')
			radar_width  = radar.get('width')

			for i in range(64):       # FIlTER BY SPEED, AGE ? 
				if radar_status[i] > 0:
				
					heading = - math.radians(radar_angle[i]) + math.pi/2
					radar_x = radar_dist[i] * math.cos(heading)
					radar_y = radar_dist[i] * math.sin(heading)

					if radar_status[i] > 0:
						if abs(radar_x) < 6 and abs(radar_y) < 40:
							ax2.plot(radar_x, radar_y, 'rx', markersize=8)
					
					
				
# ---------------------------------------------------------

		if objects != {}:


			camera_id           = objects.get('obj_id')
			camera_valid        = objects.get('valid')
			camera_x            = objects.get('x')
			camera_y            = objects.get('y')
			camera_speed        = objects.get('range_rate')
			camera_motionstatus = objects.get('motion_status')
			camera_age          = objects.get('object_age')
			camera_lane         = objects.get('object_lane')

			for i in range(8):



				if abs(camera_x[i]) > 0.1 or abs(camera_y[i]) > 0.1: # Remove all the (0,0)

					if camera_valid[i] > 0 and camera_age > 0:
						ax2.plot(-camera_y[i], camera_x[i], 'go', markersize=8)

					else:
						ax2.plot(-camera_y[i], camera_x[i], 'ro', markersize=8)

# ---------------------------------------------------------

		plt.xlim([-20, 20])
		plt.ylim([-1, 40])
		plt.grid()

		f.canvas.draw()

		plt.pause(0.001)

		r.sleep()

if __name__=='__main__':
	radar_draw_loop()
