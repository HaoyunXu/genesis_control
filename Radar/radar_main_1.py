import cv2
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError
from genesis_msgs.msg import ESRTrackReport
from genesis_msgs.msg import MandoObjectReport




img = None


# Create dictionary for radar targets
radar = dict()
radar = {}

# Create dictionary for camera targets
objects = dict()
objects = {}

# Create publishers
pub_radar  = rospy.Publisher('radar_targets', Float32MultiArray, queue_size=3)
pub_camera = rospy.Publisher('camera_targets', Float32MultiArray, queue_size=3)

def parseImg(msg):
 	global img
 	try:
 		img = CvBridge().imgmsg_to_cv2(msg, "rgb8")
 	except CvBridgeError as e:
 		print e

def parseRadar(msg):
	global radar


	status        = msg.status        # > 0 = valid target
	mode          = msg.mode          # > 0 = detected by either MR/LR
	dist          = msg.range
	vel           = msg.range_rate
	acc           = msg.range_accel
	angle         = msg.angle
	width         = msg.width        # m
	rolling_count = msg.rolling_count


	radar['status']  = status         # New_Coasted_Target(7), Invalid_Coasted_Target(6), Merged_Target(5), Coasted_Target(4),
                                          # Updated_Target(3), New_Updated_Target(2), New_Target(1), No_Target(0)
	radar['mode']          = mode
	radar['dist']          = dist
	radar['vel']           = vel      # m/s
	radar['acc']           = acc
	radar['angle']         = angle
	radar['width']         = width
	radar['rolling_count'] = rolling_count

def parseObject(msg):
	global objects


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
 	objects['motion_status'] = motion_status    # stationary (4), stopped(3), oncoming (2), preceding(1), undecided(0)
 	objects['object_age']    = object_age
 	objects['object_lane']   = object_lane


def radar_draw_loop():
	rospy.init_node('radar_camera_targets')

	rospy.Subscriber('/image_raw', Image, parseImg, queue_size = 2)
	rospy.Subscriber('/mando_radar/esr_track', ESRTrackReport, parseRadar, queue_size = 2)
	rospy.Subscriber('/mando_camera/object_detection', MandoObjectReport, parseObject, queue_size = 2)

	r = rospy.Rate(20.0)

# ---------------------------------------------

	mat1 = Float32MultiArray()
	mat1.layout.dim.append(MultiArrayDimension())
	mat1.layout.dim.append(MultiArrayDimension())
	mat1.layout.dim[0].label = "height"
	mat1.layout.dim[1].label = "width"
	mat1.layout.dim[0].size = 3
	mat1.layout.dim[1].size = 3
	mat1.layout.dim[0].stride = 3*3
	mat1.layout.dim[1].stride = 3
	mat1.layout.data_offset = 0
	#mat.data = [0]*9

	# save a few dimensions:
	dstride0_1 = mat1.layout.dim[0].stride
	dstride1 = mat1.layout.dim[1].stride
	offset1 = mat1.layout.data_offset


# ---------------------------------------------

	mat2 = Float32MultiArray()
	mat2.layout.dim.append(MultiArrayDimension())
	mat2.layout.dim.append(MultiArrayDimension())
	mat2.layout.dim[0].label = "height"
	mat2.layout.dim[1].label = "width"
	mat2.layout.dim[0].size = 3
	mat2.layout.dim[1].size = 3
	mat2.layout.dim[0].stride = 3*3
	mat2.layout.dim[1].stride = 3
	mat2.layout.data_offset = 0
	#mat.data = [0]*9

	# save a few dimensions:
	dstride0_2 = mat1.layout.dim[0].stride
	dstride2 = mat2.layout.dim[1].stride
	offset2 = mat2.layout.data_offset



# ---------------------------------------------------------
	#f, (ax1,ax2) = plt.subplots(2)
	#plt.ion()
# ---------------------------------------------------------

	global img, radar, objects

	while not rospy.is_shutdown():


		if img is None:
			print('Waiting for img')
			r.sleep()
			continue

		img = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
		#ax1.imshow(img)

		#ax2.clear()


# ------------------------------------------------------

		global list_targets_radar
		list_targets_radar = []

		if radar != {}:

			radar_status = radar.get('status')
			radar_mode   = radar.get('mode')
			radar_dist   = radar.get('dist')
			radar_vel    = radar.get('vel')
			radar_acc    = radar.get('acc')
			radar_angle  = radar.get('angle')
			radar_width  = radar.get('width')


			for i in range(64):       # FIlTER BY SPEED, STATUS ?
				if radar_status[i] > 0:

					heading = - math.radians(radar_angle[i]) + math.pi/2
					radar_x = radar_dist[i] * math.cos(heading)
					radar_y = radar_dist[i] * math.sin(heading)

					if radar_status[i] > 0:
						if abs(radar_x) < 6 and abs(radar_y) < 40:
							#ax2.plot(radar_x, radar_y, 'rx', markersize=8)
							#plt.text(radar_x+2, radar_y + 2, radar_vel[i], fontsize=10)
							#plt.text(radar_x+2, radar_y + 5, radar_status[i], fontsize=10)
							# --------------------------------
							# For publishing

							list_targets_radar.append(radar_x)
							list_targets_radar.append(radar_y)
							list_targets_radar.append(radar_vel[i])
							list_targets_radar.append(2.0) # Ex: 2.0 : unknown

			mat1.data = list_targets_radar
			pub_radar.publish(mat1)
			#r.sleep() # NEED THIS ?



# ---------------------------------------------------------

		global list_targets_camera
		list_targets_camera = []

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


				#print 'camera_motionstatus = ', camera_motionstatus[i]	
				print 'camera_speed = ', camera_speed
							
				if abs(camera_x[i]) > 0.1 or abs(camera_y[i]) > 0.1: # Remove all the (0,0)

					if camera_valid[i] > 0 and camera_age > 0:

						


						#ax2.plot(-camera_y[i], camera_x[i], '.', markersize=8)
						list_targets_camera.append(-camera_y[i])
						list_targets_camera.append(camera_x[i])
						list_targets_camera.append(camera_speed[i])
						list_targets_camera.append(1.0)
						list_targets_camera.append(camera_age[i])
						list_targets_camera.append(camera_lane[i])

					else:
						#ax2.plot(-camera_y[i], camera_x[i], '.', markersize=8)
						list_targets_camera.append(-camera_y[i])
						list_targets_camera.append(camera_x[i])
						list_targets_camera.append(camera_speed[i])
						list_targets_camera.append(2.0)
						list_targets_camera.append(camera_age[i])
						list_targets_camera.append(camera_lane[i])

			mat2.data = list_targets_camera
			pub_camera.publish(mat2)
			#r.sleep()  # Need this ?
# ---------------------------------------------------------

		#plt.xlim([-20, 20])
		#plt.ylim([-1, 40])
		#plt.grid()

		#f.canvas.draw()

		#plt.pause(0.001)

		r.sleep()

if __name__=='__main__':
	radar_draw_loop()
