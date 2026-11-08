# 电池管理（BMS）状态
float32 battery_voltage    # V
float32 charger_voltage    # V
float32 current            # A，正为充电、负为放电
float32 power              # W
float32 soc                # %
uint8 state                # 0=idle 1=discharging 2=charging 3=full 4=low_battery 5=over_current
bool charge_enabled
float32 current_offset     # A，电流零漂校正值，直接叠加到换算并取反后的电流上
