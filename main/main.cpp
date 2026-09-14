#include "bus_servo.hpp"
#include "esplog.hpp"
#include "micro_ros.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"

#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 总线舵机测试任务：在 0~240 度之间往复转动，每次到位后读取实际位置并打印
static void bus_servo_test_task(void *arg) {
  uint8_t id = 0;
  if (BusServo.discover_id(id)) {
    esplog::info("Bus servo found, id: {}", id);
  } else {
    esplog::warn("Bus servo not found, use default id: {}", BusServo.id());
  }

  constexpr float kAngleStep = 10.0f;
  constexpr uint16_t kMoveTimeMs = 500;
  float angle = BusServoClass::MIN_ANGLE_DEG;
  float direction = 1.0f;
  while (true) {
    angle += direction * kAngleStep;
    if (angle >= BusServoClass::MAX_ANGLE_DEG) {
      angle = BusServoClass::MAX_ANGLE_DEG;
      direction = -1.0f;
    } else if (angle <= BusServoClass::MIN_ANGLE_DEG) {
      angle = BusServoClass::MIN_ANGLE_DEG;
      direction = 1.0f;
    }

    if (!BusServo.move_to_angle(angle, kMoveTimeMs)) {
      esplog::warn("move_to_angle({:.1f}) failed", angle);
    }
    // 等待转动到位再读取
    vTaskDelay(pdMS_TO_TICKS(kMoveTimeMs + 200));

    uint16_t position = 0;
    if (BusServo.read_position(position)) {
      esplog::info("set {:.1f} deg -> read {} ({:.1f} deg)", angle, position, BusServo.position_to_angle(position));
    } else {
      esplog::warn("read_position failed, last status: {}", static_cast<int>(BusServo.last_response_status()));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

extern "C" void app_main(void) {
  esplog::init(esplog::level::info);

  // 初始化任务看门狗（超时 3 秒，超时触发 panic）
  // 注意：不要 esp_task_wdt_add(NULL) 把 main 任务加入监控，
  // 因为 app_main 马上返回、main 任务退出，不会再喂狗，会误触发看门狗。
  // 各常驻任务（imu_sample / motion_task / micro_ros_task）在各自线程内
  // 自行 esp_task_wdt_add + esp_task_wdt_reset 管理。
  esp_task_wdt_config_t wdt_cfg = {
      .timeout_ms = 3000,      // 3 秒超时
      .idle_core_mask = 0,     // 不监控 idle 任务
      .trigger_panic = true,   // 超时时触发 panic
  };
  esp_task_wdt_init(&wdt_cfg);

  // 底盘电机控制（UART1 驱动电机控制板）
  Motion.begin();
  esplog::info("Motion initialized");

  // IMU 初始化并启动采样线程（begin 内部创建采样线程）
  Imu.begin();
  esplog::info("Imu initialized");

  // 总线舵机（UART2 驱动）并启动往复转动测试任务
  BusServo.begin();
  xTaskCreate(bus_servo_test_task, "bus_servo_test", 1024 * 4, nullptr, 4, nullptr);

  // micro-ROS 通信：WiFi 连接 + Agent 重连 + IMU 发布/速度指令订阅
  // RosNode 已在全局构造时绑定 Imu 引用
  RosNode.begin();

  // app_main 到此返回；Imu 为全局实例常驻，Motion/RosNode 各自在独立线程运行。
}
