#Command.msg — 控制指令

float32 yaw     # 目标偏航角（让云台往左往右转多少）
float32 pitch   # 目标俯仰角（让云台往上往下转多少）
bool control    # 是否开启控制使能（true=驱动云台电机）
bool shoot      # 是否开火

#视觉系统给云台的「最终输出」。计算完弹道补偿后，把修正过的角度和射击信号发过去。




# Command.msg — Gimbal Control Command

#float32 yaw     # Target yaw angle (rad)
#float32 pitch   # Target pitch angle (rad)
#bool control    # Gimbal motor enable (true = active)
#bool shoot      # Fire control trigger (true = shoot)

# VISION SYSTEM OUTPUT:
# The definitive output from the vision system. After ballistics 
# calculations, it sends compensated angles and shooting signals to the gimbal.