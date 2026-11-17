# joystick 使用说明

本目录是 OrangePi `~/workspace/joystick/joystick_py`（master 分支 Python 版）的可编辑副本，`doc/joystick_*.tar.gz` 是历史归档快照；
改动以本目录为准，部署与消息定义同步步骤见第 8 节。

本目录用于启动底盘手柄遥控，主要文件：

| 文件 | 作用 |
| --- | --- |
| `joystick_teleop.py` | 手柄节点：发布 `chassis_msgs/velcmd`、`chassis_msgs/servo_cmd`，订阅 ESP32 上报的状态 |
| `joystick_manual.sh` | 手动启动入口，内部调用 `ros2 launch`（可透传参数） |
| `setup.sh` | 开机自启管理：`install` / `remove` / `help`，生成并管理 `joystick_teleop.service` |
| `README.md` | 本说明 |

启动链路：

```
ros2 launch joystick_bringup joystick.launch.py
  ├─ micro_ros_agent    serial /dev/ttyS4 -b 115200
  └─ joystick_teleop.py  agent 启动 2 秒后再启动
```

launch 包位于 `~/workspace/ros/src/joystick_bringup`，两个进程都配置了 `respawn=True, respawn_delay=3.0`，任一崩溃会自动重启。

## 1. 环境

- ROS 2 Humble：`/opt/ros/humble/setup.bash`
- 自定义消息工作区：`~/workspace/ros/install/setup.bash`（内含 `chassis_msgs`、`micro_ros_agent`）
- 串口 `/dev/ttyS4`，用户需属于 `dialout` 组；手柄 `/dev/input/js*`，用户需属于 `input` 组
- 非交互 shell 不会自动读 `.bashrc`，所以手动运行前要显式 `source`，服务里已由 systemd 处理

## 2. 手动使用

> 若开机自启服务正在运行，先执行 `sudo systemctl stop joystick_teleop.service`（或 `./setup.sh remove`），否则 `/dev/ttyS4` 会被占用。

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros/install/setup.bash
ros2 launch joystick_bringup joystick.launch.py

# 或者用兼容入口（效果相同）
cd ~/workspace/joystick/joystick_py
./joystick_manual.sh
```

按 `Ctrl-C` 结束，launch 会同时停掉 agent 和 teleop。

### 可覆盖的 launch 参数

```bash
ros2 launch joystick_bringup joystick.launch.py --show-args

ros2 launch joystick_bringup joystick.launch.py serial_dev:=/dev/ttyS5
ros2 launch joystick_bringup joystick.launch.py baudrate:=115200 startup_delay:=2.0
ros2 launch joystick_bringup joystick.launch.py teleop_script:=/path/to/other_teleop.py
./joystick_manual.sh serial_dev:=/dev/ttyS5
```

| 参数 | 默认值 |
| --- | --- |
| `serial_dev` | `/dev/ttyS4` |
| `baudrate` | `115200` |
| `teleop_script` | `/home/orangepi/workspace/joystick/joystick_py/joystick_teleop.py` |
| `startup_delay` | `2.0`（秒，等 agent 就绪） |

### 手柄操作

- 上电默认 `enabled=False`，按 **START（button 11）** 切换使能，再按一次关闭
- button 6 / 8：底盘速度档位 `+` / `-`
- button 7 / 9：舵机 `move_time_ms` `+` / `-`
- 未使能、急停或手柄掉线时不下发 `velcmd` 和 `servo_cmd`
- 摇杆居中且底盘上报的线速度、角速度都为 0 时，不再重复下发零速 `velcmd`
- 同时按 **X（button 3）+ Y（button 4）** 进入/保存电流零漂标定，详见第 8 节

### 确认链路正常

```bash
ros2 topic list | grep chassis_msgs
ros2 topic echo /chassis_msgs/motion
ros2 topic echo /chassis_msgs/servo_status
```

正常应能看到 `/chassis_msgs/{velcmd,servo_cmd,servo_status,imu,xl9535,ir,motion,battery}`；agent 日志里出现 `create_topic` / `create_datareader` 表示 ESP32 已连上。

### 关于屏幕显示

手动运行时终端会每 50 ms 清屏刷新状态。后台服务通过环境变量 `JOYSTICK_DISPLAY=0` 关闭该输出，避免刷满 journal；`joystick_teleop.py` 默认仍为显示，只有显式设为 `0` 才静默。

## 3. 开机自动启动

`setup.sh` 负责生成 systemd 服务 `joystick_teleop.service`（unit 文件
`/etc/systemd/system/joystick_teleop.service`），设为开机自启并立即启动。
它不依赖登录，开机进入 `multi-user.target` 即启动。

```bash
cd ~/workspace/joystick/joystick_py
./setup.sh install
```

`install` 依次做这些事：

1. 检查 `joystick_manual.sh`、`joystick_teleop.py` 存在；`/dev/ttyS4` 不存在时只告警不中断；
2. 停止并禁用遗留的 `joystick_bringup.service`（旧 ROS2 launch 服务），避免两个
   `micro_ros_agent` 抢串口；
3. 停止可能已由 `joystick_cpp` 安装的同名 `joystick_teleop.service`；
4. 生成 unit 文件，关键配置：
   - `User=orangepi`、`Group=orangepi`、`WorkingDirectory=.../joystick_py`
   - `ExecStart=<本目录>/joystick_manual.sh teleop_script:=<本目录>/joystick_teleop.py`
   - `Environment=JOYSTICK_DISPLAY=0` 及 SDL 相关变量（后台不刷屏、SDL 无显示器可用）
   - `Restart=on-failure`、`RestartSec=3`
   - `WantedBy=multi-user.target`
5. `systemctl daemon-reload` 后 `systemctl enable --now joystick_teleop.service`，并打印服务状态。

unit 文件要写 `/etc/systemd/system/`，脚本会在需要时调用 `sudo`，按提示输入密码即可。
`./setup.sh help` 可查看用法；不带参数等价于 `help`。

> **`joystick_py` 与 `joystick_cpp` 共用服务名 `joystick_teleop.service`**，后一次
> `install` 会先停掉当前服务、覆盖 unit 文件再启动；两者都要占用 `/dev/ttyS4`，不能同时运行。
> 切到 C++ 版前先在本目录执行 `./setup.sh remove`。

常用查看命令：

```bash
systemctl is-enabled joystick_teleop.service     # enabled / disabled
systemctl is-active  joystick_teleop.service     # active / inactive
systemctl status joystick_teleop.service --no-pager
journalctl -u joystick_teleop.service -f         # 实时日志
journalctl -u joystick_teleop.service -n 100 --no-pager
pgrep -af 'micro_ros_agent|joystick_teleop.py'
```

修改了 `joystick_teleop.py` 或 launch 文件后：

```bash
sudo systemctl restart joystick_teleop.service
```

若改动了 launch 包（`package.xml` / `CMakeLists.txt` / launch 文件），先重新编译再重启：

```bash
cd ~/workspace/ros
source /opt/ros/humble/setup.bash
colcon build --packages-select joystick_bringup
source install/setup.bash
sudo systemctl restart joystick_teleop.service
```

## 4. 关掉自动开机启动（保留服务）

只取消开机自启、同时停掉当前运行：

```bash
sudo systemctl disable --now joystick_teleop.service
```

只想临时停一下、不取消自启（下次开机仍会自动启动）：

```bash
sudo systemctl stop joystick_teleop.service
```

之后想恢复：

```bash
sudo systemctl enable --now joystick_teleop.service
```

此时 unit 文件仍在 `/etc/systemd/system/` 下，`joystick_manual.sh` / `ros2 launch` 手动启动方式也照常可用。

## 5. 删除自动开机启动的服务（彻底移除）

```bash
cd ~/workspace/joystick/joystick_py
./setup.sh remove
```

`remove` 会 `stop` + `disable` 服务、删除 `/etc/systemd/system/joystick_teleop.service`、
`daemon-reload` 并 `reset-failed`；unit 文件不存在时会跳过删除而不报错。

如需手工执行，等价命令为：

```bash
sudo systemctl disable --now joystick_teleop.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/joystick_teleop.service
sudo systemctl daemon-reload
sudo systemctl reset-failed joystick_teleop.service 2>/dev/null || true
systemctl status joystick_teleop.service --no-pager   # 应提示 Unit could not be found
```

删除服务后，仍可用第 2 节的方式手动 `ros2 launch` 启动。

如果连 launch 包也不再需要（可选，删除后将无法用 `ros2 launch joystick_bringup ...`）：

```bash
rm -rf ~/workspace/ros/src/joystick_bringup \
       ~/workspace/ros/build/joystick_bringup \
       ~/workspace/ros/install/joystick_bringup
cd ~/workspace/ros && colcon build
```

## 6. 常见故障

| 现象 | 排查 |
| --- | --- |
| agent 报串口占用 / 打不开 | `pgrep -af micro_ros_agent`，确认服务和手动启动没有同时运行；`ls -l /dev/ttyS4`；确认 `joystick_cpp` 的服务没在跑 |
| 串口权限不足 | `groups orangepi` 应含 `dialout`、`input`；缺失时 `sudo usermod -aG dialout,input orangepi` 并重新登录 |
| 手柄识别不到 | `ls -l /dev/input/js*`；看 `journalctl -u joystick_teleop.service` 里的 pygame 日志 |
| ESP32 没连上 | 检查 `/dev/ttyS4`、波特率 115200、接线；agent 正常时日志有 `create_topic` / `create_datareader` |
| 话题列表为空 | 确认已 `source ~/workspace/ros/install/setup.bash`，且 `ROS_DOMAIN_ID` 与服务端一致 |
| 改了脚本没生效 | `sudo systemctl restart joystick_teleop.service` |
| 日志被刷屏 | 确认服务里 `Environment=JOYSTICK_DISPLAY=0` 未被删掉 |
| `./setup.sh` 报权限错误 | `chmod +x setup.sh joystick_manual.sh`；写 unit 需要 `sudo`，按提示输入密码 |
| `install` 后服务起不来 | `systemctl status joystick_teleop.service`、`journalctl -u joystick_teleop.service -n 100`，确认 `joystick_bringup` 已编译且旧服务已停用 |
| 服务与 C++ 版互相顶掉 | 两版共用 `joystick_teleop.service`，只保留一个：先 `./setup.sh remove`，再在目标目录 `./setup.sh install` |

## 7. 相关文件位置

- 自启管理脚本：`~/workspace/joystick/joystick_py/setup.sh`
- 服务定义：`/etc/systemd/system/joystick_teleop.service`（由 `setup.sh install` 生成）
- 遗留服务：`/etc/systemd/system/joystick_bringup.service`（`install` 时会停止并禁用）
- launch 文件：`~/workspace/ros/src/joystick_bringup/launch/joystick.launch.py`
- 手柄脚本：`~/workspace/joystick/joystick_py/joystick_teleop.py`
- 手动入口：`~/workspace/joystick/joystick_py/joystick_manual.sh`
- C++ 版：`~/workspace/joystick/joystick_cpp/`，同样用 `setup.sh` 管理同名服务
- 备份：`joystick.sh.bak`（改名前的旧入口）、`joystick_teleop.py.bak*`

## 8. 设置电流零漂（BMS 电流标定）

### 8.1 背景与符号约定

INA240 + ADC 采样链路每块板的零点都不一样，直接按 `I = (Vout - Vref) / (gain × Rsense)`
换算出的电流会带一个固定偏差。ESP32 中的最终电流为：

```
I_final = -1 × (Vout - Vref) / (gain × Rsense) + current_offset
```

其中 `-1` 把硬件极性校正为「充电为正、放电为负」，`current_offset`（单位 A）就是本流程要标定的
零漂值。它保存在 ESP32 的 NVS 命名空间 `bms`、键 `curr_offset` 中，掉电不丢，并随
`chassis_msgs/BatteryData` 每 1 秒上报一次，供上位机回读。

标定前需要：

1. ESP32 已烧录带零漂功能的固件，且 `extra_ros_packages/chassis_msgs/msg/BatteryData.msg`
   与 OrangePi `~/workspace/ros/src/chassis_msgs/msg/BatteryData.msg` 都含
   `float32 current_offset` 字段；
2. OrangePi 侧已重新编译消息包并重启服务（见 8.5）；
3. 设备处于**稳定放电**状态，负载固定，标定过程中不要启停电机；
4. 万用表串联在电池回路中，能读到稳定的放电电流；
5. `ros2 topic echo /chassis_msgs/battery` 能看到数据持续刷新。

### 8.2 按键

| 按键 | button | 作用 |
| --- | --- | --- |
| X + Y 同时按 | button3 + button4 | 进入零漂标定模式；再次按下则把当前值发给 ESP32 并保存 |
| B | button1 | 零漂值 `+0.1 A` |
| A | button0 | 零漂值 `-0.1 A` |

### 8.3 操作步骤

1. **进入模式**：同时按下 X + Y，屏幕出现 `[Battery offset calibration]` 区块。
2. **自动清零**：程序先读 `BatteryData.current_offset`。若不为 0，自动向
   `chassis_msgs/battery_current_offset`（`std_msgs/Float32`）发送 `0.0`，把 ESP32 零漂清 0；
   程序会等到下一帧 `BatteryData.current_offset` 回读为 0 才进入下一步。
   这一步保证后续看到的是未经补偿的原始电流。
3. **测量并计算**：确认上报零漂已为 0 后，程序提示：
   - 用万用表读取设备当前**真实放电电流**，记为 **A1**（例如 `2.00 A`；表笔接法不同时
     也可能显示负值，计算时一律取绝对值 `|A1|`）；
   - 在屏幕 `[Battery: bms]` 区读取程序显示的电流，记为 **A2**（此时零漂已清 0，即未经
     补偿的原始值，通常例如 `-2.30 A`）；**A2 是带符号的数，直接代入，不要取绝对值**；
   - ESP32 的电流关系是 `当前屏幕显示值 + 零漂值 = 校正后的正确显示值`，而正确显示值
     应为 `-|A1|`（放电为负），因此：

     ```
     零漂值 = (|A1| × -1) - A2 = -|A1| - A2
     ```

     例如 `A1 = 2.00 A`、`A2 = -2.30 A`：
     `零漂值 = -2.00 - (-2.30) = +0.30 A`。
     若程序读数偏小，如 `A2 = -1.70 A`：
     `零漂值 = -2.00 - (-1.70) = -0.30 A`。
4. **调整本地目标值**：按 **B（button1）** 每次 `+0.1 A`，按 **A（button0）** 每次 `-0.1 A`，
   把屏幕上的 `target` 调到上一步算出的目标零漂值。此阶段只改上位机本地值，不影响 ESP32。
5. **下发并保存**：再次同时按下 **X + Y**，程序把当前 `target` 作为 `std_msgs/Float32`
   发布到 `chassis_msgs/battery_current_offset`。ESP32 收到后立即写入 NVS 并生效，
   之后每帧 `BatteryData` 都会带回该值。
6. **确认**：退出模式后观察屏幕 `esp32 offset` 是否等于刚下发的值；再用万用表复核放电电流，
   `BatteryData.current` 的绝对值应与万用表读数一致。重启设备后再次回读，确认 NVS 保存成功。

### 8.4 手工验证命令

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/ros/install/setup.bash

# 查看当前电流与零漂
ros2 topic echo /chassis_msgs/battery

# 手工设置零漂（例如 0.3 A）
ros2 topic pub --once /chassis_msgs/battery_current_offset std_msgs/msg/Float32 "{data: 0.3}"

# 清零
ros2 topic pub --once /chassis_msgs/battery_current_offset std_msgs/msg/Float32 "{data: 0.0}"
```

### 8.5 部署同步

ESP32 侧新增 msg 字段后增量构建不会自动更新，需要重建 micro-ROS 静态库：

```bash
cd <esp32 工程目录>
idf.py fullclean
# 按所用 micro-ROS 组件说明清理 libmicroros.a / include / micro_ros_src / micro_ros_dev
idf.py build flash
```

OrangePi 侧把本目录的 `joystick_teleop.py`、`joystick_manual.sh`、`setup.sh`、`README.md`
同步到 `~/workspace/joystick/joystick_py`（`setup.sh` 需 `chmod +x`），
并确认 `~/workspace/ros/src/chassis_msgs/msg/BatteryData.msg` 末行包含：

```
float32 current_offset     # A，电流零漂校正值，直接叠加到换算并取反后的电流上
```

然后重新编译消息包并重启服务：

```bash
cd ~/workspace/ros
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_msgs
source install/setup.bash
sudo systemctl restart joystick_teleop.service
```

若尚未安装开机自启服务，用 `cd ~/workspace/joystick/joystick_py && ./setup.sh install` 代替 `restart`。

### 8.6 注意事项

- 零漂是**加性**偏差，本流程只校正零点；若不同电流下误差成比例变化，说明还存在增益误差，
  需要两点校准（同时标定 gain 与 offset）。
- 零漂值必须按 `-|A1| - A2` 用**带符号**的 A2 计算。不要写成 `|A2| - |A1|`：该式只在
  A2 为负时数值碰巧相同，一旦零漂大到让屏幕显示值变正就会算错。
- 标定与读数时负载必须稳定，万用表与程序读数尽量同时读取；电流纹波大时可多读几次取平均。
- 零漂会随温度、INA240 `Vref`、ADC 参考电压漂移，温度明显变化后建议重新标定。
- 零漂值会影响 `current`、安时积分 SOC、充放电状态判定与过流判定，设置过大可能误触发保护。
- 每块板的零漂不同，标定值保存在**该板 NVS**，换板或擦除 NVS 后需重新标定。
