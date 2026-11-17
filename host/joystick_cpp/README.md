# joystick_cpp（自定义串口版手柄节点）

本目录是 `feature/custom-serial` 分支的上位机手柄代码，用 C++17 + spdlog 实现，
通过自定义二进制串口协议直接与 ESP32 通信；不依赖 ROS2、`micro_ros_agent` 或 `chassis_msgs`。
master 分支的 Python 版在同级目录 `../joystick_py/`，两者**互斥**（都要占用 `/dev/ttyS4`）。

## 1. 文件

| 文件 | 说明 |
| --- | --- |
| `serial_protocol.hpp` | 帧格式 / 消息 ID / payload / CRC8 / 解析器，与 ESP32 `main/serial_protocol.hpp` 内容完全相同 |
| `joystick_teleop.cpp` | 手柄读取（Linux joystick API）、串口收发、20Hz 控制循环、状态显示、spdlog 日志 |
| `CMakeLists.txt` | 构建配置（spdlog + Threads） |
| `setup.sh` | 开机自启管理：`install` / `remove` / `help` |
| `build/joystick_teleop` | 构建产物，服务默认执行它 |

## 2. 依赖与构建

- g++、cmake、spdlog（OrangePi 上已装 1.9.2）
- 串口 `/dev/ttyS4` @115200，手柄 `/dev/input/js0`
- **ESP32 必须烧录 `feature/custom-serial` 分支固件**；master 固件跑 micro-ROS，协议不兼容

```bash
cd ~/workspace/joystick/joystick_cpp
cmake -S . -B build
cmake --build build -j$(nproc)
```

`setup.sh install` 会在可执行文件不存在时自动执行上述构建。

## 3. 手动运行

```bash
./build/joystick_teleop                                    # 默认 /dev/ttyS4、js0
./build/joystick_teleop --serial /dev/ttyUSB0 --joy 1 --no-display
```

| 参数 | 说明 |
| --- | --- |
| `--serial PATH` | 串口设备，默认 `/dev/ttyS4` |
| `--joy N` | 手柄编号，对应 `/dev/input/jsN`，默认 0 |
| `--no-display` | 关闭 20Hz 清屏显示 |

环境变量 `JOYSTICK_DISPLAY=0` 等效于 `--no-display`（后台服务里已设置）。
所有日志与屏幕输出都经 spdlog：事件日志走默认 logger（带时间戳/级别），
屏幕状态显示走独立 logger、`%v` pattern，避免清屏内容被日志前缀打乱。

## 4. 开机自启（setup.sh）

```bash
./setup.sh install   # 生成 /etc/systemd/system/joystick_teleop.service，enable --now 启动
./setup.sh remove    # stop + disable + 删除服务文件 + daemon-reload
./setup.sh help      # 用法说明
```

- 服务名 `joystick_teleop.service` 与 `../joystick_py/setup.sh` **共用**：后一次 `install`
  会先停掉当前服务、覆盖单元文件再启动，实现版本切换；不要同时启用两个版本。
- `install` 会停止并禁用遗留的 `joystick_bringup.service`（旧 ROS2 launch 服务），
  避免 `micro_ros_agent` 抢占串口。
- 服务以 `orangepi` 用户运行，`Restart=on-failure`、`RestartSec=3`；
  `ExecStart` 为 `build/joystick_teleop --serial /dev/ttyS4 --joy 0`。

常用检查：

```bash
systemctl status joystick_teleop.service
journalctl -u joystick_teleop.service -f
```

## 5. 通信协议

帧格式：`A5 5A | msg_id(u8) | len(u8) | payload[len] | crc8(u8)`，
CRC8（多项式 `0x8C`）覆盖 `msg_id + len + payload`，payload 为小端 POD 结构。

| 方向 | ID | 内容 | 频率 |
| --- | --- | --- | --- |
| 上位机 → ESP32 | `0x01` | velcmd（linear、angular） | 20Hz（零速且底盘已停时不发） |
| 上位机 → ESP32 | `0x02` | servo_cmd（angle_deg、move_time_ms） | 推杆有输入时 20Hz |
| 上位机 → ESP32 | `0x03` | battery_current_offset，单位 A | 标定时 |
| 上位机 → ESP32 | `0x04` | ping | 500ms 保活 |
| ESP32 → 上位机 | `0x10`~`0x14` | imu / servo_status / xl9535 / ir / motion | 10Hz |
| ESP32 → 上位机 | `0x15` | battery | 1Hz |
| ESP32 → 上位机 | `0x16` | pong | ping 应答 |

## 6. 与 Python 版功能对照

保留：START 使能/禁用、速度档位、急停、十字键 + 摇杆死区与缩放、舵机开环积分与
`move_time_ms` 调整、X+Y 进入/保存零漂标定、B/A ±0.1A 微调与自动清零回读、
六类状态分区显示、零速抑制下发、500ms 速度看门狗（ESP32 侧）。

差异：通信层换成自定义串口帧；手柄从 pygame(SDL) 换成 Linux joystick API
（十字键按 `ABS_HAT0X/Y` 识别）；日志与屏幕输出改用 spdlog。

## 7. 切换回 master 版

```bash
cd ~/workspace/joystick/joystick_cpp && ./setup.sh remove
cd ~/workspace/joystick/joystick_py && ./setup.sh install
```

同时需把 ESP32 烧回 master 分支固件（micro-ROS），否则协议不匹配。
