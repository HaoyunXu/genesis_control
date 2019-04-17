Header header

time stamp1					# timestamp for the Valid1 part; same as header.stamp
time stamp2					# timestamp for the Valid2 part

# Long Range (Valid1)
int32 lr_sn                                     # ?, probably a boolean for if the sensor works?
float32 lr_range_rate                           # m/s
float32 lr_range                                # m
float32 lr_range_power                          # dB
float32 lr_range_angle                          # deg

# Medium Range (Valid2)
int32 mr_sn                                     # ?, probably a boolean for if the sensor works?
float32 mr_range_rate                           # m/s
float32 mr_range                                # m
float32 mr_range_power                          # dB
float32 mr_range_angle                          # deg



