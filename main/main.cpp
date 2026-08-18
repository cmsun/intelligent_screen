#include "esplog.hpp"
#include "micro_ros_node.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"

#include <memory>
#include <sstream>

#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

// IMU 融合/日志线程：栈 20K，避免 main 栈溢出
constexpr uint32_t kImuTaskStackBytes = 20 * 1024;
constexpr UBaseType_t kImuTaskPriority = 5;

constexpr float kLogPeriodS = 0.5f; // 5 Hz 文本日志

void imu_task(void *arg) {
  auto *imu = static_cast<imu::MPU9250 *>(arg);

  float last = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
  float last_log = 0.0f;

  while (true) {
    const float now = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
    const float dt = now - last;
    last = now;

    // 高频融合（~200 Hz），micro_ros_node 线程按需读取 last_sample()
    imu::ImuSample s;
    if (imu->read(s, dt) != ESP_OK) {
      // 读取失败不刷屏，5s 提示一次
      static float last_err = 0.0f;
      if (now - last_err >= 5.0f) {
        last_err = now;
        esplog::error("MPU-9250 read failed");
      }
    }

    // 降低文本日志频率，避免拖慢融合循环
    if (now - last_log >= kLogPeriodS) {
      last_log = now;
      const imu::ImuSample &ls = imu->last_sample();
      const imu::Vec3 e = imu->euler_deg();
      esplog::info("accel={:.2f},{:.2f},{:.2f} gyro={:.2f},{:.2f},{:.2f} "
                   "mag={:.2f},{:.2f},{:.2f} "
                   "temp={:.2f} | roll={:.1f} pitch={:.1f} yaw={:.1f}",
                   ls.accel.x, ls.accel.y, ls.accel.z, ls.gyro.x, ls.gyro.y, ls.gyro.z,
                   ls.mag.x, ls.mag.y, ls.mag.z, ls.temperature, e.x, e.y, e.z);
      // fmt v11 不支持通过 operator<< 隐式格式化自定义类型，先转成字符串
      std::ostringstream motion_str;
      motion_str << Motion;
      esplog::info("{}", motion_str.str());
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz 循环
  }
}

} // namespace

extern "C" void app_main(void) {
  esplog::init(esplog::level::info);

  // 默认 Config 已按 GPIO6/7/8/9/10 接线（SDO/SDI/SCK/CS/INT）
  // 芯片倒焊：Z 轴重力方向反号（静止 az=-9.5），需翻转符号。
  // X/Y 轴符号待实测确认后一并修正。
  imu::MPU9250::Config imu_cfg{};
  imu_cfg.mag_fusion_enabled = true; // 启用磁力计融合，锁住 yaw 航向，消除漂移

  auto imu = std::make_unique<imu::MPU9250>(imu_cfg);
  if (imu == nullptr) {
    esplog::error("Failed to allocate MPU9250 instance");
    return;
  }
  if (auto err = imu->init(); err != ESP_OK) {
    esplog::error("MPU-9250 init failed: {}", esp_err_to_name(err));
    return;
  }
  esplog::info("MPU-9250 initialized");

  // 底盘电机控制（UART1 驱动电机控制板）
  Motion.begin();
  esplog::info("Motion initialized");

  // micro-ROS 通信：WiFi 连接 + Agent 重连 + IMU 发布/速度指令订阅，独立线程中运行
  RosNode.begin(imu.get());

  TaskHandle_t imu_task_handle = nullptr;
  const BaseType_t rc =
      xTaskCreate(imu_task, "imu_task", kImuTaskStackBytes, imu.get(),
                  kImuTaskPriority, &imu_task_handle);
  if (rc != pdPASS) {
    esplog::error("Failed to create IMU task (stack={} bytes)", kImuTaskStackBytes);
    return;
  }

  // app_main 到此返回，融合/日志在独立线程中持续进行，micro-ROS 在独立线程中发布
}
