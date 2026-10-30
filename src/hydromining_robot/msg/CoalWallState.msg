# world 中固定初始墙面坐标系；层按 layer_names 顺序、行优先展开。
std_msgs/Header header
geometry_msgs/Pose wall_frame
float64 resolution
uint32 width
uint32 height
string[] layer_names
float32[] data
bool uncertainty_calibrated
string assessment
float64 fraction_lower
float64 fraction_upper
float64 coverage
