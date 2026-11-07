# 总线舵机状态，由 ESP32 周期上报。
# ready 为 true 表示已成功读到过位置，上位机据此确认舵机可用。
bool ready
uint8 id
int32 position
float32 angle_deg
float32 voltage_v
