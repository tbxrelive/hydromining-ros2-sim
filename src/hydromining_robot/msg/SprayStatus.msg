# All geometry is in world; pressure is MPa. Effects are uncalibrated.
std_msgs/Header header
uint64 shot_id
string state
float64 pressure
float64 duration
float64 elapsed
geometry_msgs/Point nozzle_position
geometry_msgs/Vector3 direction
bool has_hit
geometry_msgs/Point hit_position
geometry_msgs/Vector3 hit_normal
float64 effect_strength
string message
