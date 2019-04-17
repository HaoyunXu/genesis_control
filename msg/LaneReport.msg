Header header

time stampA					# timestamp for the "A"-part of the message; same as header.stamp
time stampB					# timestamp for the "B"-part of the message

# Lane polynomial in vehicle frame is y = a0 + a1 x + a2 x^2 + a3 x^3
float32 a0
float32 a1
float32 a2
float32 a3

# Mark types: 6 "Invalid" 5 "Botts'Dots" 4 "DoubleLaneMark" 3 "RoadEdge" 2 "Undecided" 1 "Solid" 0 "Dashed" ;
int32 lane_mark_type
# Quality types: 0, 1 "Low Quality" 2, 3 "High Quality"
int32 lane_mark_quality
