#ImuData.msg — 惯性测量数据

float32 roll      # 绕前向轴的旋转角（弧度）
float32 pitch     # 绕右向轴的旋转角
float32 yaw       # 绕上轴的旋转角
float32 gyro_x    # x轴角速度（弧度/秒），变化速率
float32 gyro_y    # y轴角速度
float32 gyro_z    # z轴角速度
float32 accel_x   # x轴加速度（米/秒²）
float32 accel_y   # y轴加速度
float32 accel_z   # z轴加速度

#云台一边转、一边发自身姿态,用姿态角补偿子弹弹道，，，，，将我这个汉语改成英语



# ImuData.msg - Inertial Measurement Unit Data

# Euler angles in radians (Post-processing feedback)
#float32 roll      # Rotation around the forward axis (Roll)
#float32 pitch     # Rotation around the rightward axis (Pitch)
#float32 yaw       # Rotation around the upward axis (Yaw)

# Angular velocity (rad/s)
#float32 gyro_x    
#float32 gyro_y    
#float32 gyro_z    

# Linear acceleration (m/s²)
#float32 accel_x   
#float32 accel_y   
#float32 accel_z   

# GIMBAL CONTROL LOGIC:
# The gimbal broadcasts its real-time attitude while in motion.
# These attitude angles are utilized for bullet trajectory compensation.