#include "battery.hpp"
#include "bus_servo.hpp"
#include "esplog.hpp"
#include "ir_receiver.hpp"
#include "led.hpp"
#include "micro_ros.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"
#include "setup_motor_id.hpp"
#include "xl9535.hpp"

#include <esp_err.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

extern "C" void app_main(void) {
  // 初始化任务看门狗（超时 3 秒，超时触发 panic）
  // 注意：不要 esp_task_wdt_add(NULL) 把 main 任务加入监控：app_main 末尾改为常驻循环，
  // 但未被 add 的任务不受 task WDT 约束，循环内 vTaskDelay 会正常让出 CPU。
  // 各常驻任务（imu_sample / motion_task / micro_ros_task）在各自线程内
  // 自行 esp_task_wdt_add + esp_task_wdt_reset 管理。
  // esp_task_wdt_config_t wdt_cfg = {
  //     .timeout_ms = 3000,      // 3 秒超时
  //     .idle_core_mask = 0,     // 不监控 idle 任务
  //     .trigger_panic = true,   // 超时时触发 panic
  // };
  // esp_task_wdt_init(&wdt_cfg);

  esplog::init(esplog::level::debug);

  // 电流零漂标定值存放在 NVS，需在所有模块 begin() 之前初始化 NVS 分区
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // 分区满或版本升级：擦除后重试（会清掉已标定的零漂值，需重新标定）
    nvs_flash_erase();
    nvs_err = nvs_flash_init();
  }
  if (nvs_err != ESP_OK) {
    esplog::error("NVS init failed: {}", static_cast<int>(nvs_err));
  }

  // 设置电机控制板的ID
  // SetupMotorID.begin(2);
  // 底盘电机控制（UART1 驱动电机控制板）
  // 手柄遥控必须启动：begin 内部完成电机使能/速度模式/断连保护，并创建控制任务
  Motion.begin();

  // XL9535 GPIO 扩展：风扇输出、地面检测输入、头顶/下巴障碍物检测输入
  // begin 内部完成 I2C 初始化、方向配置并创建轮询任务（上报数据需要它持续刷新输入缓存）
  if (!Xl9535.begin())
  {
    esplog::error("XL9535 init failed");
  }

  // 红外接收：左右两路 HS-S23P（GPIO13 / GPIO14）
  // begin 内部创建两个 RMT 接收通道和 NEC 解码任务，收到有效帧会打印日志
  if (!IrReceiver.begin())
  {
    esplog::error("IR receiver init failed");
  }

  // IMU 初始化并启动采样线程（begin 内部创建采样线程）
  Imu.begin();

  // 电池管理：begin 内部完成 ADC 初始化、用电压初始化 SOC，并创建周期采样任务
  if (!Battery.begin())
  {
    esplog::error("Battery init failed");
  }

  // 总线舵机（UART2 驱动）：micro-ROS 上报 ServoStatus，接收 ServoCmd 目标角度与转动时间（驱动层限位 0~170 deg）
  BusServo.begin();

  // micro-ROS 通信（传输方式由 Kconfig 决定，当前为串口 UART0 / 115200）
  // Agent 重连 + IMU/ServoStatus 发布 + velcmd/servo_cmd 订阅（OrangePi 手柄节点）
  // RosNode 运行时直接读取全局 Imu 实例
  RosNode.begin();

  // app_main 不再返回：每 1s 依据电池充放电状态设置 LED 闪烁节奏。
  // 放电：亮 100ms / 灭 900ms（慢闪）；充电：亮 100ms / 灭 100ms（快闪）。
  // led_start_blink() 在闪烁线程已存在时只更新时长，不会重复创建线程。
  BatteryClass::State last_state = BatteryClass::State::idle;
  while (true)
  {
    const BatteryClass::State state = Battery.state();
    if (state != last_state)
    {
      if (state == BatteryClass::State::charging)
      {
        Led.led_start_blink(50, 50);
      }
      else
      {
        Led.led_start_blink(50, 950);
      }
      last_state = state;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
