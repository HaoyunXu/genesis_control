Header header

int32[] index

int32[] alive_count_a 
int32[] object_identifier
int32[] object_age
int32[] object_valid			# valid(1) or invalid (0)
int32[] object_mode			# cutout (2), cutin (1), neither(0)
int32[] object_lane			# same lane (0), next left (1), far left (2), near right (3), far right (4), unknown (5)
float32[] angle_rate			# deg
float32[] angle_left			# deg
float32[] angle_right			# deg
int32[] cipv				# cipv = closest in path vehicle(1) or not (0)
int32[] brake_lights			# on (1), off or N/A (0)
int32[] blinker_info			# both(4), right(3), left(2), off(0)

int32[] alive_count_b
float32[] range				# m
float32[] range_rate			# m/s
int32[] motion_status			# stationary (4), stopped(3), oncoming (2), preceding(1), undecided(0)
float32[] accel_x			# m/s^2
float32[] position_y			# m




