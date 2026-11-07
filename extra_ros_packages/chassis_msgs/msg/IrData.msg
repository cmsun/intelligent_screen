# 红外接收：最近一帧 NEC 解码结果 + 左右通道近期是否有红外活动
uint8 side               # 0=左，1=右
uint8 address            # 8 位地址
uint8 command            # 8 位命令
uint32 raw_code          # 32 位原始码
bool address_valid       # 反码校验通过
bool repeat              # 长按重复帧
uint32 timestamp_ms      # 收到时刻（ESP32 毫秒时钟，0 表示尚无帧）
bool left_active         # 最近 500ms 内左侧有红外活动
bool right_active
