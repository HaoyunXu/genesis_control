import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from genesis_msgs.msg import ESRTrackReport
from genesis_msgs.msg import MandoObjectReport
import rospy
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import math
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox

pub_ped = rospy.Publisher('ped_bb', BoundingBoxArray, queue_size=10)
pub_car = rospy.Publisher('car_bb', BoundingBoxArray, queue_size=10)
pub_follow = rospy.Publisher('follow', BoundingBoxArray, queue_size=10)

def parseBoxes(data):
	global pub_ped
	global pub_car
	data_ped = BoundingBoxArray()
	data_car = BoundingBoxArray()
	data_follow = BoundingBoxArray()
	counterp = 0
	counterc = 0
	data_ped.header = data.header
	data_car.header = data.header
	data_follow.header = data.header

	for i in range(len(data.boxes)):
		x = data.boxes[i].pose.position.x
		y = data.boxes[i].pose.position.y
		height = data.boxes[i].dimensions.z
		length = data.boxes[i].dimensions.x
		width = data.boxes[i].dimensions.y
		minl = 1000;
		minr = 1000;
		minf = 1000;
		if (y<50 and y>0) and (x<2.5 and x>-2.5):
			# # Vehicle
			# if (height < 5 and height > 0.5) and (length < 20 and length > 0) and (width < 20 and width > 0):
			data_follow.boxes.append(data.boxes[i])
		if (y<20 and y>-10) and (x<8 and x>-8):
			height = data.boxes[i].dimensions.z
			length = data.boxes[i].dimensions.x
			width = data.boxes[i].dimensions.y
			# Pedstrian
			if (height < 1.8 and height > 0.5) and (length < 1.5 and length > 0) and (width < 1.5 and width > 0):
				data_ped.boxes.append(data.boxes[i])
				counterp = counterp + 1
			elif (height < 5 and height > 0.5) and (length < 10 and length > 0) and (width < 10 and width > 0):
				data_car.boxes.append(data.boxes[i])
				counterc = counterc + 1
			if x>0 and x<minr and y<10 and y>-10:
				minr = x
			if x<0 and abs(x)<minl and y<10 and y>-10
				minl = abs(x)
			if y>0 and y<minf and x<10 and x>10
				minf = y


	pub_ped.publish(data_ped)
	pub_car.publish(data_car)
	pub_follow.publish(data_follow)


def radar_draw_loop():
	rospy.init_node('lidar_box_filter', anonymous = True)
	rospy.Subscriber('/bounding_boxes', BoundingBoxArray, parseBoxes, queue_size = 2)

	rospy.spin()
if __name__=='__main__':
	radar_draw_loop()
