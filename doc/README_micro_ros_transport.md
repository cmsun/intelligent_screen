# micro-ROS 通信方式切换说明

本工程支持两种 micro-ROS 与 Agent 的通信方式：

- **串口通信**（custom transport，通过 UART0）
- **WiFi 通信**（UDP over WiFi，默认方式）

本文档详细说明两种方式的切换方法、背后的原理以及常见问题的排查。

---

## 一、核心原理（务必先读）

micro-ROS 的传输方式由**两套相互独立的配置**共同决定，这是理解切换方法的关键：

| 层次 | 配置文件 | 生成的宏 | 作用 |
|------|----------|----------|------|
| **库层** | `app-colcon.meta` | `RMW_UXRCE_TRANSPORT_CUSTOM` / `RMW_UXRCE_TRANSPORT_UDP` | 决定 `libmicroros.a` 内部使用哪种传输，**同时决定 `custom_transport.h` 是否被包含** |
| **应用层** | 组件 Kconfig（menuconfig） | `CONFIG_MICRO_ROS_ESP_UART_TRANSPORT` / `CONFIG_MICRO_ROS_ESP_NETIF_WLAN` | 决定应用层编译哪些代码（串口回调 或 WiFi 初始化） |

### 为什么需要两个配置

`components/micro_ros/include/rmw_microros/rmw_microros.h` 中有这样的条件包含：

```c
#ifdef RMW_UXRCE_TRANSPORT_CUSTOM
#include <rmw_microros/custom_transport.h>
#endif
```

而 `RMW_UXRCE_TRANSPORT_CUSTOM` 这个宏，是在 colcon 构建 `libmicroros.a` 时根据 `app-colcon.meta` 中的 `-DRMW_UXRCE_TRANSPORT=custom` 烧进头文件的。

**两者必须配套使用**，缺一不可：

| 想用的方式 | menuconfig 选择 | `app-colcon.meta` |
|-----------|-----------------|-------------------|
| **串口** | `Micro XRCE-DDS over UART` | **必须存在**，内容为 `custom` |
| **WiFi** | `WLAN interface` | **必须删除或改名** |

### 常见错误：两套配置不匹配

如果 menuconfig 选了串口，但 `app-colcon.meta` 不存在（库仍是 UDP 版），编译会报：

```
error: 'rmw_uros_options_set_custom_transport' was not declared in this scope;
did you mean 'rmw_uros_options_set_udp_address'?
```

**原因**：应用层代码走进了串口分支去调用 `rmw_uros_options_set_custom_transport()`，但库是 UDP 版，`RMW_UXRCE_TRANSPORT_CUSTOM` 未定义，`custom_transport.h` 没有被包含，函数自然未声明。

---

## 二、切换到串口通信

### 步骤 1：创建 `app-colcon.meta`

在工程根目录创建 `app-colcon.meta`，内容如下：

```json
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_TRANSPORT=custom"
            ]
        }
    }
}
```

### 步骤 2：menuconfig 选择串口

```bash
idf.py menuconfig
```

操作路径：

```
micro-ROS Settings
  └─ micro-ROS network interface select
       ├─ ( ) WLAN interface
       ├─ ( ) Ethernet interface
       └─ (X) Micro XRCE-DDS over UART      ← 选这个
```

选择后，下方会出现 **UART Settings** 菜单，可配置引脚：

| 配置项 | 说明 | 默认值 |
|--------|------|--------|
| UART TX pin | 发送引脚，`-1` 表示不修改 | `-1` |
| UART RX pin | 接收引脚，`-1` 表示不修改 | `-1` |
| UART RTS pin | RTS 引脚，`-1` 表示不修改 | `-1` |
| UART CTS pin | CTS 引脚，`-1` 表示不修改 | `-1` |

> 默认值 `-1` 表示保持 ESP32 默认引脚映射（UART0 默认 TX=GPIO43, RX=GPIO44），通常无需修改。

### 步骤 3：修改串口端口与波特率（可选）

组件 Kconfig **不提供** UART 端口号和波特率配置，这两个参数在代码中以常量定义：

文件：`main/esp32_serial_transport.h`

```c
#ifndef MICRO_ROS_SERIAL_UART_NUM
#define MICRO_ROS_SERIAL_UART_NUM (0)          // UART 端口号，0 = UART0
#endif

#ifndef MICRO_ROS_SERIAL_BAUDRATE
#define MICRO_ROS_SERIAL_BAUDRATE (115200)     // 波特率，需与 agent 端一致
#endif
```

如需使用其他串口或波特率，直接修改这两个常量即可。

### 步骤 4：删除旧的库构建产物（**关键**）

传输方式改变后，`libmicroros.a` 必须重建：

```bash
cd /home/scm/workspace/esp32_ws/intelligent_screen

rm -rf components/micro_ros/libmicroros.a \
       components/micro_ros/include \
       components/micro_ros/micro_ros_src \
       components/micro_ros/micro_ros_dev
```

### 步骤 5：编译烧录

```bash
idf.py fullclean
idf.py build flash
```

> 此步耗时较长（需重新运行 colcon 编译整个 micro-ROS 库，约 10-20 分钟），属正常现象。

### 步骤 6：验证宏已切换

```bash
grep "RMW_UXRCE_TRANSPORT" components/micro_ros/include/rmw_microxrcedds_c/config.h
```

**期望输出**：

```
/* #undef RMW_UXRCE_TRANSPORT_UDP */
/* #undef RMW_UXRCE_TRANSPORT_TCP */
/* #undef RMW_UXRCE_TRANSPORT_SERIAL */
#define RMW_UXRCE_TRANSPORT_CUSTOM
```

看到 `RMW_UXRCE_TRANSPORT_CUSTOM` 即为成功。

### 步骤 7：主机启动 Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

- `--dev`：ESP32 在主机上的串口设备节点（也可能是 `/dev/ttyACM0` 等）
- `-b`：波特率，**必须与 `MICRO_ROS_SERIAL_BAUDRATE` 一致**

查看设备节点：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

---

## 三、切换回 WiFi 通信

### 步骤 1：删除 `app-colcon.meta`

```bash
rm -f app-colcon.meta
```

建议使用改名代替删除，方便日后切回串口：

```bash
mv app-colcon.meta app-colcon.meta.serial.bak
```

### 步骤 2：menuconfig 选择 WiFi 并填写参数

```bash
idf.py menuconfig
```

操作路径：

```
micro-ROS Settings
  └─ micro-ROS network interface select
       ├─ (X) WLAN interface          ← 选这个
       ├─ ( ) Ethernet interface
       └─ ( ) Micro XRCE-DDS over UART
```

> **重要**：选择 UART 时，WiFi 相关菜单会被 Kconfig 隐藏并从 `sdkconfig` 中移除，因此切换回 WiFi 后**必须重新填写**以下参数，否则会使用默认值导致连接失败。

**WiFi Configuration** 菜单：

| 配置项 | 说明 | 示例值 |
|--------|------|--------|
| WiFi SSID | 路由器名称 | `CDKJ` |
| WiFi Password | WiFi 密码 | `cdkj20101228` |
| Maximum retry | 最大重连次数 | `2147483647` |

**micro-ROS Agent IP / Port** 菜单（位于 `micro-ROS Settings` 下）：

| 配置项 | 说明 | 示例值 |
|--------|------|--------|
| micro-ROS Agent IP | 运行 agent 的主机 IP | `192.168.110.17` |
| micro-ROS Agent Port | agent 监听端口 | `8888` |

> Agent IP 必须是**主机在 ESP32 所连 WiFi 网段内的 IP**，可用 `hostname -I` 查询。

### 步骤 3：删除旧的库构建产物

```bash
rm -rf components/micro_ros/libmicroros.a \
       components/micro_ros/include \
       components/micro_ros/micro_ros_src \
       components/micro_ros/micro_ros_dev
```

### 步骤 4：编译烧录

```bash
idf.py fullclean
idf.py build flash
```

### 步骤 5：验证宏已切换

```bash
grep "RMW_UXRCE_TRANSPORT" components/micro_ros/include/rmw_microxrcedds_c/config.h
```

**期望输出**：

```
#define RMW_UXRCE_TRANSPORT_UDP
/* #undef RMW_UXRCE_TRANSPORT_CUSTOM */
```

### 步骤 6：主机启动 Agent

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

## 四、切换流程速查表

```
┌─────────────────────────────────────────────────────────────┐
│                    切换传输方式（通用流程）                    │
├─────────────────────────────────────────────────────────────┤
│  1. 处理 app-colcon.meta                                     │
│     串口 → 创建（内容 =custom）                              │
│     WiFi → 删除 / 改名                                       │
│                                                              │
│  2. idf.py menuconfig                                        │
│     串口 → Micro XRCE-DDS over UART                          │
│     WiFi → WLAN interface + 填写 SSID/密码/Agent IP          │
│                                                              │
│  3. 删除库构建产物（必做）                                    │
│     rm -rf components/micro_ros/{libmicroros.a,include,       │
│            micro_ros_src,micro_ros_dev}                       │
│                                                              │
│  4. idf.py fullclean && idf.py build flash                    │
│                                                              │
│  5. grep RMW_UXRCE_TRANSPORT                                  │
│     components/micro_ros/include/rmw_microxrcedds_c/config.h  │
│     确认宏与预期一致                                          │
│                                                              │
│  6. 启动对应方式的 agent                                      │
│     串口 → serial --dev /dev/ttyUSB0 -b 115200                │
│     WiFi → udp4 --port 8888                                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 五、为什么必须删除库构建产物

`components/micro_ros/` 目录下这些是**构建产物**，而非源码：

| 路径 | 说明 |
|------|------|
| `components/micro_ros/include/` | 生成的头文件，含已固化的传输宏 |
| `components/micro_ros/libmicroros.a` | 预编译的 micro-ROS 静态库 |
| `components/micro_ros/micro_ros_src/` | colcon 源码工作区 |
| `components/micro_ros/micro_ros_dev/` | colcon 工具链环境 |

底层 `libmicroros.mk` 检测到这些目录已存在时会**跳过重新构建**（增量构建），导致 `app-colcon.meta` 的改动不生效。

> **注意**：`idf.py fullclean` **不会**删除这些文件（它们在组件目录内，不在 `build/` 目录里），因此必须手动删除。

---

## 六、常见问题排查

### 问题 1：编译报 `rmw_uros_options_set_custom_transport was not declared`

**原因**：两套配置不匹配——menuconfig 选了串口，但库仍是 UDP 版。

**排查**：

```bash
grep "RMW_UXRCE_TRANSPORT" components/micro_ros/include/rmw_microxrcedds_c/config.h
```

**解决**：确认 `app-colcon.meta` 存在且内容为 `custom`，然后删除库构建产物重新编译。

### 问题 2：串口模式下反复断开重连，agent 报 `deserialization error`

**现象**：

```
error | InputMessage.cpp | deserialization error | buffer:
0000: 00 00 00 00 00 00 00 00 00 00 00 00 00
```

**原因**：串口 0 被 ESP-IDF console 日志和 micro-ROS 同时使用，日志字节混入 XRCE 数据帧导致解析失败。

**解决**：将 console 输出改走其他通道：

```bash
idf.py menuconfig
# Component config → ESP System Settings → Channel for console output
# → 选择 "USB Serial/JTAG Controller" 或 "None"
```

若板子仅有一个 USB-TTL 串口（如 CP210x/CH340，表现为 `/dev/ttyUSB0`），只能选 **None** 彻底关闭日志输出。

### 问题 3：WiFi 模式下 ESP32 连不上 agent

**排查顺序**：

1. 确认 agent 已监听端口：
   ```bash
   ss -ulnp | grep 8888
   ```

2. 确认主机 IP 与 `sdkconfig` 中配置一致：
   ```bash
   hostname -I
   grep CONFIG_MICRO_ROS_AGENT_IP sdkconfig
   ```

3. 确认 ESP32 与主机在同一网段（查看串口日志中的 `got ip:`）

4. 检查防火墙是否拦截 UDP：
   ```bash
   sudo ufw status
   sudo ufw allow 8888/udp
   ```

5. 抓包确认 UDP 包是否到达主机：
   ```bash
   sudo tcpdump -i any udp port 8888 -n
   ```

> 注意：主机能 ping 通 ESP32 只说明 ICMP 通，与 UDP 8888 无关，不能作为判断依据。

### 问题 4：切换后编译很慢

正常现象。切换传输方式会触发 colcon 重新编译整个 micro-ROS 库，耗时约 10-20 分钟。后续普通增量编译不受影响。

---

## 七、相关代码文件

| 文件 | 说明 |
|------|------|
| `main/micro_ros.cpp` | 根据 `CONFIG_MICRO_ROS_ESP_UART_TRANSPORT` 在编译期选择 transport |
| `main/esp32_serial_transport.c/.h` | 串口 custom transport 的四个回调函数实现 |
| `app-colcon.meta` | 控制库层传输方式（串口模式必需） |
| `sdkconfig` | menuconfig 的配置结果，含 WiFi/串口参数 |
| `components/micro_ros/Kconfig.projbuild` | 组件自带的传输方式选项定义 |

### 应用层条件编译示例

`main/micro_ros.cpp` 中根据组件宏选择不同代码路径：

```cpp
#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
    // 串口传输：注册自定义 transport 回调
    _serial_port = static_cast<size_t>(MICRO_ROS_SERIAL_UART_NUM);
    rmw_uros_options_set_custom_transport(true, (void *)&_serial_port,
                                          esp32_serial_open, esp32_serial_close,
                                          esp32_serial_write, esp32_serial_read, rmw_options);
#else
    // WiFi 传输：连接 WiFi 并配置 Agent 的 IP 与端口
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                     CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
#endif
```

---

## 八、验证通信是否正常

切换完成并启动 agent 后，可通过以下命令验证：

```bash
# 查看话题列表
ros2 topic list

# 查看 IMU 数据（本工程发布的话题）
ros2 topic echo /chassis_msgs/imu

# 查看发布频率
ros2 topic hz /chassis_msgs/imu

# 测试速度指令订阅（本工程订阅的话题）
ros2 topic pub /chassis_msgs/velcmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

串口模式下 agent 成功建立连接的日志示例：

```
info | Root.cpp          | create_client       | create              | client_key: 0x7DA8AD9B
info | SessionManager    | establish_session   | session established | client_key: 0x7DA8AD9B
info | ProxyClient.cpp   | create_participant  | participant created | participant_id: 0x000(1)
info | ProxyClient.cpp   | create_datawriter   | datawriter created  | datawriter_id: 0x000(5)
```

若日志中 `session established` 反复出现，说明连接在不断断开重连，请参考**问题 2** 排查。
