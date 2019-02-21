Please run this in a new terminal before playing the rosbag. The topic image_raw will then be published. 

rosrun image_transport republish compressed in:=/image_compressed raw out:=/image_raw
