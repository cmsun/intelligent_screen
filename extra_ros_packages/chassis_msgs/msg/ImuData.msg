# IMU 姿态：只传欧拉角与 yaw 轴角速度（不传四元数/加速度/协方差）
# 角度单位 deg，yaw 为罗盘航向（右转为正），yaw_rate 取自陀螺仪实测值
float32 roll_deg
float32 pitch_deg
float32 yaw_deg
float32 yaw_rate_deg_s
