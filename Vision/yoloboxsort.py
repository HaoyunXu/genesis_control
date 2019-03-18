import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from genesis_msgs.msg import ESRTrackReport
from genesis_msgs.msg import MandoObjectReport
import rospy
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import math
#from jsk_recognition_msgs.msg import BoundingBoxArray
from autoware_msgs.msg import image_obj

#yolobox = rospy.Publisher('yolo_bb', BoundingBoxArray, queue_size=10)

img = None; img_lock = False; img_tm = None
pub_img_proc = rospy.Publisher("/image_yolo", Image, queue_size=2)

def img_callback(msg):
	# Maybe downsample?
	global img, img_tm, img_lock
	if img_lock == False:
		try:
			img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
			img_tm = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
		except CvBridgeError as e:
			print e


def parseYoloBoxes(msg):
	global img, img_lock
	#global yolobox
	#print 2
	#print data.obj.x
	#pubbox.publish(yolo_box)
	if len(msg.obj) > 0:
		print msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

		img_local = None
		# copy the image, get the lock
		if (img is not None) and (img_lock == False):
			img_lock = True
			img_local = np.copy(img)
			img_lock = False
		else:
			print 'Waiting for image.'


		# overlay the boxes on the image and publish
		if img_local is not None:
			color = (0,0,0)
		 	if msg.type == 'person':
				color = (255,0,0)
			else:
				color = (0,0,255)

			for obj in msg.obj:
				x = obj.x
				y = obj.y
				w = obj.width
				h = obj.height
				s = obj.score
				print 'x: %d, y: %d, h: %d, w: %d, score: %f\n' % (x,y,h,w,s)
				cv2.rectangle(img_local, (x,y), (x+w, y+h), color = color)
			img_msg = CvBridge().cv2_to_imgmsg(img_local, encoding="bgr8")
			pub_img_proc.publish(img_msg)

def radar_draw_loop():
	rospy.init_node('lidar_box_filter', anonymous = True)
	sub_person = rospy.Subscriber('/obj_person/image_obj', image_obj, parseYoloBoxes, queue_size = 2)
	sub_car = rospy.Subscriber('/obj_car/image_obj', image_obj, parseYoloBoxes, queue_size = 2)
	sub_img = rospy.Subscriber("/image_raw", Image, img_callback, queue_size=2)
	rospy.spin()

if __name__=='__main__':
	radar_draw_loop()
