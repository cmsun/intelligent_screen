#include "esplog.hpp"
#include "mpu9250.hpp"

#include <cmath>

#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

namespace {

// IMU 读取线程：栈大小 10K，避免 main 栈溢出
constexpr uint32_t kImuTaskStackBytes = 10 * 1024; // 10K
constexpr UBaseType_t kImuTaskPriority = 5;

void imu_task(void *arg) {
  // 由调用方 new 传入，线程结束时释放
  auto *imu = static_cast<imu::MPU9250 *>(arg);

  float last = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
  float last_print = 0.0f;
  while (true) {
    const float now = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
    const float dt = now - last;
    last = now;

    imu::ImuSample s{};
    if (imu->read(s, dt) == ESP_OK) {
      // 降低打印频率（~10Hz），避免拖慢融合循环、影响 dt 精度
      if (now - last_print >= 0.1f) {
        last_print = now;
        const imu::Vec3 e = imu->euler_deg();
        esplog::info("accel={:.2f},{:.2f},{:.2f} gyro={:.2f},{:.2f},{:.2f} "
                     "mag={:.2f},{:.2f},{:.2f} "
                     "temp={:.2f} | roll={:.1f} pitch={:.1f} yaw={:.1f}",
                     s.accel.x, s.accel.y, s.accel.z, s.gyro.x, s.gyro.y,
                     s.gyro.z, s.mag.x, s.mag.y, s.mag.z, s.temperature, e.x,
                     e.y, e.z);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // ~100 Hz
  }
}

} // namespace

extern "C" void app_main(void) {
  // // 初始化 NVS 分区
  // nvs_flash_init();
  // // 初始化事件循环
  // esp_event_loop_create_default();
  // // 初始化网络接口
  // esp_netif_init();
  // // 启用 DHCP 服务器‌
  // esp_netif_create_default_wifi_ap();
  // // 启用 DHCP 客户端以支持 IP 地址获取及上层 TCP/IP 通信‌
  // esp_netif_create_default_wifi_sta();
  esplog::init(esplog::level::info);

  // 默认 Config 已按 GPIO6/7/8/9/10 接线（SDO/SDI/SCK/CS/INT）
  // 芯片倒焊：Z 轴重力方向反号（静止 az=-9.5），需翻转符号。
  // X/Y 轴符号待实测确认后一并修正。
  imu::MPU9250::Config imu_cfg{};
  imu_cfg.accel_sign.z = -1.0f;
  imu_cfg.gyro_sign.z = -1.0f; // 倒焊时陀螺仪 Z 也同步反号
  imu_cfg.beta = 0.8f;         // 增大梯度下降增益，加快收敛（原 0.1 偏慢）
  // 静态零偏校准：垫平静止实测 roll=14.2 pitch=1.1，减去使其归零
  imu_cfg.roll_offset_deg = 14.2f;
  imu_cfg.pitch_offset_deg = 1.1f;
  // 在堆上创建，传入 IMU 线程，避免占用 app_main 的栈
  auto *imu = new (std::nothrow) imu::MPU9250(imu_cfg);
  if (imu == nullptr) {
    esplog::error("Failed to allocate MPU9250 instance");
    return;
  }
  if (auto err = imu->init(); err != ESP_OK) {
    esplog::error("MPU-9250 init failed: {}", esp_err_to_name(err));
    delete imu;
    return;
  }
  esplog::info("MPU-9250 initialized, starting read thread");

  // 创建独立线程运行 IMU 读取，栈 10K
  TaskHandle_t imu_task_handle = nullptr;
  const BaseType_t rc = xTaskCreate(imu_task, "imu_task", kImuTaskStackBytes,
                                    imu, kImuTaskPriority, &imu_task_handle);
  if (rc != pdPASS) {
    esplog::error("Failed to create IMU task (stack={} bytes)",
                  kImuTaskStackBytes);
    delete imu;
    return;
  }

  // app_main 到此返回，IMU 读取在独立线程中持续进行
}
