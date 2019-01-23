# Object identifier
#  - The identifier is assumed to be unique within a topic.
#  - There is no identifier correllation to be assumed between different object tracker topics.
#  - Same identifier on samples of a single topic referes to the same object.
int32 object_id

# The object's estimated motion
float64 x
float64 y
float64 vx
float64 vy

# Flag to indicate if the track is moving or not
bool is_moving

# Age of the track (in cycles)
int32 age

# The distance the object has moved. The main feature in determining 
# whether the object has moved.
float64 distance_moved

# Dominant classification of the tracked object. Currently unused.
# Object classification can change over the lifetime of an object.
uint8 classification
float32[8] class_values
uint8 CLASS_Unclassified=0
uint8 CLASS_UnknownSmall=1
uint8 CLASS_UnknownBig=2
uint8 CLASS_Pedestrian=3
uint8 CLASS_Bike=4
uint8 CLASS_Car=5
uint8 CLASS_Truck=6
