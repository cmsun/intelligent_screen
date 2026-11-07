#include "battery.hpp"
#include "bus_servo.hpp"
#include "esplog.hpp"
#include "ir_receiver.hpp"
#include "micro_ros.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"
#include "setup_motor_id.hpp"
#include "xl9535.hpp"

#include <esp_task_wdt.h>

extern "C" void app_main(void) {
  esplog::init(esplog::level::debug);

  // 初始化任务看门狗（超时 3 秒，超时触发 panic）
  // 注意：不要 esp_task_wdt_add(NULL) 把 main 任务加入监控，
  // 因为 app_main 马上返回、main 任务退出，不会再喂狗，会误触发看门狗。
  // 各常驻任务（imu_sample / motion_task / micro_ros_task）在各自线程内
  // 自行 esp_task_wdt_add + esp_task_wdt_reset 管理。
  // esp_task_wdt_config_t wdt_cfg = {
  //     .timeout_ms = 3000,      // 3 秒超时
  //     .idle_core_mask = 0,     // 不监控 idle 任务
  //     .trigger_panic = true,   // 超时时触发 panic
  // };
  // esp_task_wdt_init(&wdt_cfg);

  // 设置电机控制板的ID
  // SetupMotorID.begin(2);
  // 底盘电机控制（UART1 驱动电机控制板）
  // 手柄遥控必须启动：begin 内部完成电机使能/速度模式/断连保护，并创建控制任务
  // Motion.begin();

  // XL9535 GPIO 扩展：风扇输出、地面检测输入、头顶/下巴障碍物检测输入
  // begin 内部完成 I2C 初始化、方向配置并创建轮询任务（上报数据需要它持续刷新输入缓存）
  // if (!Xl9535.begin())
  // {
  //   esplog::error("XL9535 init failed");
  // }

  // 红外接收：左右两路 HS-S23P（GPIO13 / GPIO14）
  // begin 内部创建两个 RMT 接收通道和 NEC 解码任务，收到有效帧会打印日志
  // if (!IrReceiver.begin())
  // {
  //   esplog::error("IR receiver init failed");
  // }

  // IMU 初始化并启动采样线程（begin 内部创建采样线程）
  // Imu.begin();

  // 电池管理：begin 内部完成 ADC 初始化、用电压初始化 SOC，并创建周期采样任务
  // if (!Battery.begin())
  // {
  //   esplog::error("Battery init failed");
  // }

  // 总线舵机（UART2 驱动）：micro-ROS 上报 ServoStatus，接收 ServoCmd 目标角度与转动时间（驱动层限位 0~170 deg）
  BusServo.begin();

  // micro-ROS 通信（传输方式由 Kconfig 决定，当前为串口 UART0 / 115200）
  // Agent 重连 + IMU/ServoStatus 发布 + velcmd/servo_cmd 订阅（OrangePi 手柄节点）
  // RosNode 已在全局构造时绑定 Imu 引用
  RosNode.begin();

  // app_main 到此返回；Imu 为全局实例常驻，Motion/RosNode 各自在独立线程运行。
}
