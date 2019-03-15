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
from autoware_detection_msgs.msg import DetectedObjectArray

pubbox = rospy.Publisher('yolo_bb', BoundingBoxArray, queue_size=10)

def parseYoloBoxes(data):
	global pubbox
	data_copy = data
	counter = 0
	for i in range(len(data.boxes)):
		x = data.boxes[i].pose.position.x
		y = data.boxes[i].pose.position.y
		if (y<20 and y>-10) and (x<10 and x>-10):
			# Pedstrian
			height = data.boxes[i].dimensions.z
			length = data.boxes[i].dimensions.x
			width = data.boxes[i].dimensions.y
			if (height < 1.8 and height > 0.5) and (length < 1.5 and length > 0) and (width < 1.5 and width > 0):
				data_copy.boxes[counter] = data.boxes[i]
				counter = counter + 1

	for j in range(len(data.boxes)-counter):
		data_copy.boxes.pop()

	pubbox.publish(data_copy)


def radar_draw_loop():
	rospy.init_node('lidar_box_filter', anonymous = True)
	rospy.Subscriber('/bounding_boxes', BoundingBoxArray, parseBoxes, queue_size = 2)

	rospy.spin()
if __name__=='__main__':
	radar_draw_loop()
