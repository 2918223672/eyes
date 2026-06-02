#EnemyStatusMsg.msg — 敌方状态

builtin_interfaces/Time timestamp      # 这个状态数据的更新时间
int8[] invincible_enemy_ids            # 处于无敌状态（刚复活/受击保护）的敌方ID
int32 id                               # 当前跟踪的敌方ID
float32 distance                       # 距离
float32 yaw                            # 偏航角
float32 pitch                          # 俯仰角
float32 velocity                       # 速度
bool is_tracking                       # 是否正在跟踪
int32 health                           # 敌方血量（0~100 或其他赛制值

#需要连续稳定的信息——敌方不止在哪，还包括血量、无敌状态、是否还在跟踪




# EnemyStatusMsg.msg — Enemy Status and Metadata

#builtin_interfaces/Time timestamp      # Time when the data was updated
#int8[] invincible_enemy_ids            # IDs currently in invincible state (spawn/hit protection)
#int32 id                               # Target ID currently being tracked
#float32 distance                       # Distance to target (m)
#float32 yaw                            # Target yaw relative to gimbal (rad)
#float32 pitch                          # Target pitch relative to gimbal (rad)
#float32 velocity                       # Target linear velocity (m/s)
#bool is_tracking                       # Tracking lock status
#int32 health                           # Enemy HP value

# High-continuity and stable information:
# Tracks not only the target's physical coordinates, but also its 
# biological status (HP), protection state (invincibility), and tracking persistence.