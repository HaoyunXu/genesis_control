
#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

radar_list_targets           = None
camera_list_targets          = None

previous_camera_list_targets = None
previous_radar_list_targets  = None


def callback_radar(data):
	global radar_list_targets
	radar_list_targets = data.data
	


def callback_camera(data):
	global camera_list_targets
	camera_list_targets = data.data


def main():
   	global camera_list_targets, radar_list_targets, previous_camera_list_targets, previous_radar_list_targets

	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("radar_targets", Float32MultiArray, callback_radar)
	rospy.Subscriber("camera_targets", Float32MultiArray, callback_camera)


	# ------------------------------------------------------------
	# Initialize the figure
	f, ax = plt.subplots(1)
	plt.ion()
	
	# ------------------------------------------------------------


	

	while not rospy.is_shutdown():

		# Refresh plot
		ax.clear()

		# ------------------------------------------------------------
		# If no targets detected, the global variables are not updated so 
                # they stay equal to they previous value. So if that happens, we set
                # them to None. Maybe better to do an empty tuple instead. 

		if camera_list_targets == previous_camera_list_targets:
			camera_list_targets = None

		if radar_list_targets == previous_radar_list_targets:
			radar_list_targets = None

		# ------------------------------------------------------------
		# Reshape the lists to make it easier to work. 
		# We only reshape if it's not a None, of course
		# We change the name (ex: radar_list_targets_matrix to still be able to compare with previous value at the end of the main

		radar_list_targets_matrix  = []
		camera_list_targets_matrix = []

		if radar_list_targets != None:  
			radar_list_targets_matrix  = np.array(radar_list_targets).reshape(len(radar_list_targets)/4,4)     # Number of rows and columns (x,y,speed,label)

		if camera_list_targets != None:
			camera_list_targets_matrix = np.array(camera_list_targets).reshape(len(camera_list_targets)/5, 5)  # Number of rows and columns (x,y,speed)



		# ------------------------------------------------------------
		# Here we do the fusion. We go through all the points and:
		# -> Calculate the distance between all the camera points with the radar points
                #   -> If they are close, we do an average 
		#   -> If not close, we keep them both
		# We append everything in a new matrix 	

		# LABEL: 1.0 = Car
		#        2.0 = Unknown

		all_targets = []	
		min_distance = 5 #Min distance between two points to be considered the same points
		for i in range(0, len(camera_list_targets_matrix)):        #camera targets loop
			for j in range(0, len(radar_list_targets_matrix)): #radar targets loop

				x_cam     = camera_list_targets_matrix[i][0]
				y_cam     = camera_list_targets_matrix[i][1]
				v_cam     = camera_list_targets_matrix[i][2] #Speed		
				label_cam = camera_list_targets_matrix[i][3]
				age_cam   = camera_list_targets_matrix[i][4]
			 
				x_radar     = radar_list_targets_matrix[j][0]
				y_radar     = radar_list_targets_matrix[j][1]
				v_radar     = radar_list_targets_matrix[j][2]  #Speed
				label_radar = radar_list_targets_matrix[j][3]
				
			
				distance = np.sqrt( (x_radar - x_cam)**2 + (y_radar - y_cam)**2 )
				if distance < min_distance:
					x_average = (x_cam + x_radar)/2
					y_average = (y_cam + y_radar)/2
					v_average = (v_cam + v_radar)/2
					
					# We average and add the label (1.0 = car)
					point_average = [x_average, y_average, v_average, 1.0]
					all_targets.append(point_average)
				else:

					# We add the label (1.0 = car, 2.0 = unknown)
					a = np.append(camera_list_targets_matrix[i],1.0)
					b = np.append(radar_list_targets_matrix[j],2.0)
					
					
					all_targets.append(list(a))
					all_targets.append(list(b))
					
		# ------------------------------------------------------------
		# Here we plot the targets
		# Change the indexes !
		column_of_x = [i[0] for i in all_targets]
		column_of_y = [i[1] for i in all_targets]

		ax.scatter(column_of_x, column_of_y,color='green', marker='o')

		plt.xlim([-20,20])
		plt.ylim([-1,40])
		plt.grid()
		f.canvas.draw()  # Do I need this ? 
		plt.pause(0.001) # Do I need this ?
					
		
		# ------------------------------------------------------------
		# Update the new previous global variables
		previous_camera_list_targets = camera_list_targets
		previous_radar_list_targets  = radar_list_targets
		# ------------------------------------------------------------

		rospy.sleep(0.5)
	
	rospy.spin()

if __name__ == '__main__':
    
	main()
