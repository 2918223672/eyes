#AutoaimTargetMsg.msg — 自瞄检测结果

builtin_interfaces/Time timestamp  # 时间戳：这个检测是哪个时刻拍到的
int8[] target_ids                  # 动态数组：这一帧检测到的候选目标ID列表
float32 yaw                        # 目标在相机坐标系下的偏航角（弧度）
float32 pitch                      # 目标在相机坐标系下的俯仰角
float32 distance                   # 目标距离（米）
float32 velocity                   # 目标速度（米/秒）
float32 tracking_time              # 当前目标被连续跟踪了多久（秒）
int32 id                           # 当前选择的目标ID
bool tracking                      # 是否正在跟踪中

#检测器每帧跑完，把「看到了什么」告诉跟踪器和决策层：有多少目标、各自在哪、距离多远




# AutoaimTargetMsg.msg — Auto-aim Detection Results

#builtin_interfaces/Time timestamp  # Capture timestamp of the detection frame
#int8[] target_ids                  # Dynamic array of candidate target IDs in this frame
#float32 yaw                        # Target yaw relative to camera frame (rad)
#float32 pitch                      # Target pitch relative to camera frame (rad)
#float32 distance                   # Distance to target (m)
#float32 velocity                   # Target relative velocity (m/s)
#float32 tracking_time              # Cumulative tracking duration for current target (s)
#int32 id                           # Selected primary target ID
#bool tracking                      # Tracking lock status

# SYSTEM WORKFLOW:
# After the detector completes its frame processing, it broadcasts 
# perceived data to the tracker and decision layers: how many targets 
# are visible, their respective positions, and their distances.