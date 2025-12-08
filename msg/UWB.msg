# msg/UWB.msg
std_msgs/Header header
string tag_id
float32[] dists      # 距离数组 (对应 anchor_id)
float32[] q_values   # 质量数组
int32[] anchor_ids   # 基站 ID 数组
# 下面是 UWB 附带的 IMU 数据 (可选，如果你想转发)
geometry_msgs/Vector3 acc
geometry_msgs/Vector3 gyro