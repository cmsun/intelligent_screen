#include "esplog.hpp"
#include "micro_ros_node.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"

#include <esp_task_wdt.h>

extern "C" void app_main(void) {
  esplog::init(esplog::level::info);

  // 初始化任务看门狗（超时 3 秒，超时触发 panic）
  // 注意：不要 esp_task_wdt_add(NULL) 把 main 任务加入监控，
  // 因为 app_main 马上返回、main 任务退出，不会再喂狗，会误触发看门狗。
  // 各常驻任务（imu_sample / motion_task / micro_ros_node_task）在各自线程内
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

  // micro-ROS 通信：WiFi 连接 + Agent 重连 + IMU 发布/速度指令订阅
  // RosNode 已在全局构造时绑定 Imu 引用
  RosNode.begin();

  // app_main 到此返回；Imu 为全局实例常驻，Motion/RosNode 各自在独立线程运行。
}
