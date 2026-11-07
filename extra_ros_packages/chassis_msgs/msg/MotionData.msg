# 底盘运动状态；速度单位 SI（m/s、rad/s），rpm 为电机内部单位
float32 linear_speed_cur
float32 angular_speed_cur
int16 left_motor_rpm
int16 right_motor_rpm
uint8 left_motor_state     # 0=disable 1=enable
uint8 right_motor_state
uint8 left_motor_mode      # 0=open 1=current 2=speed 3=position
uint8 right_motor_mode
bool left_communication_error
bool right_communication_error
