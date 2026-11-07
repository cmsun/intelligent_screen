# 总线舵机命令，由上位机（OrangePi 手柄节点）发布到 chassis_msgs/servo_cmd。
# angle_deg     目标角度 [deg]；通信层不做限位，统一由 BusServoClass 驱动层夹紧
# move_time_ms  本次转动的执行时间 [ms]，直接透传给 BusServo.move_to_angle()
float32 angle_deg
uint16 move_time_ms
