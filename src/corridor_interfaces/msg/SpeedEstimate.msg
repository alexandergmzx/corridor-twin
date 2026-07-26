# Camera-derived longitudinal speed. No ground-truth pose or odometry is permitted.
std_msgs/Header header
string corridor_profile
float64 station_m
float64 speed_mps
float64 speed_stddev_mps
float64 corridor_width_m
float64 speed_limit_mps
uint32 gate_from_id
uint32 gate_to_id
uint32 observation_count
bool valid
