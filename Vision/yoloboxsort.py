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

personYolo_bb = []
carYolo_bb = []
yolo_person_time = None
yolo_car_time = None

class YoloDetection():
	def __init__(self, x = None, y = None, h = None, w = None, score = None, degree = None):
		self.x = x
		self.y = y
		self.w = w
		self.h = h
		self.score = score
		self.degree = degree

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
	global personYolo_bb, carYolo_bb,yolo_person_time,yolo_car_time
	if len(msg.obj) > 0:
		print msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

		bb_array = []
		if msg.type == 'person':
			bb_array = personYolo_bb
			yolo_person_time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
		elif msg.type == 'car':
			bb_array = carYolo_bb
			yolo_car_time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

		bb_array[:] = []

		for obj in msg.obj:
			x = obj.x
			y = obj.y
			w = obj.width
			h = obj.height
			s = obj.score
			degree = np.arctan(2346/(x-644)) / 3.1415 * 180
			if s < 0.5:
				continue
			if degree < 0:
				degree = degree + 180
			print 'type: %s, x: %d, y: %d, h: %d, w: %d, score: %f, degree: %f\n' % (msg.type,x,y,h,w,s,degree)
			bb_array.append( YoloDetection(x,y,h,w,s,degree) )


def YoloMain():
	rospy.init_node('lidar_box_filter', anonymous = True)
	sub_person = rospy.Subscriber('/obj_person/image_obj', image_obj, parseYoloBoxes, queue_size = 2)
	sub_car = rospy.Subscriber('/obj_car/image_obj', image_obj, parseYoloBoxes, queue_size = 2)
	sub_img = rospy.Subscriber("/image_raw", Image, img_callback, queue_size=2)

	r = rospy.Rate(10.0)
	global img, personYolo_bb, carYolo_bb, img_tm, img_lock, yolo_person_time, yolo_car_time
	while not rospy.is_shutdown():
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
			if yolo_person_time is None or abs(yolo_person_time - img_tm) > 0.1:
				personYolo_bb[:] = []
			elif len(personYolo_bb) > 0:
 				for person in personYolo_bb:
					cv2.rectangle(img_local, (person.x,person.y), (person.x+person.w, person.y+person.h), color = (255,0,0), thickness = 3)

			if yolo_car_time is None or abs(yolo_car_time - img_tm) > 0.1:
				carYolo_bb[:] = []
			elif len(carYolo_bb) > 0:
 				for car in carYolo_bb:
					cv2.rectangle(img_local, (car.x,car.y), (car.x+car.w, car.y+car.h), color = (0,0,255), thickness = 3)
			img_msg = CvBridge().cv2_to_imgmsg(img_local, encoding="bgr8")
			pub_img_proc.publish(img_msg)

		r.sleep()

if __name__=='__main__':
	YoloMain()
