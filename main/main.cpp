#include "esplog.hpp"
#include "mpu9250.hpp"

#include <cmath>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void) {
  esplog::init(esplog::level::info, "ESP32_LOG");

  // 默认 Config 已按 GPIO6/7/8/9/10 接线（SDO/SDI/SCK/CS/INT）
  imu::MPU9250 imu(imu::MPU9250::Config{});
  if (auto err = imu.init(); err != ESP_OK) {
    esplog::error("MPU-9250 init failed: {}", esp_err_to_name(err));
    return;
  }
  esplog::info("MPU-9250 initialized, starting read loop");

  float last = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
  while (true) {
    const float now = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
    const float dt = now - last;
    last = now;

    imu::ImuSample s{};
    if (imu.read(s, dt) == ESP_OK) {
      const imu::Vec3 e = imu.euler_deg();
      esplog::info(
          "accel={:.2f},{:.2f},{:.2f} gyro={:.2f},{:.2f},{:.2f} mag={:.2f},{:.2f},{:.2f} "
          "temp={:.2f} | roll={:.1f} pitch={:.1f} yaw={:.1f}",
          s.accel.x, s.accel.y, s.accel.z, s.gyro.x, s.gyro.y, s.gyro.z, s.mag.x,
          s.mag.y, s.mag.z, s.temperature, e.x, e.y, e.z);
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // ~100 Hz
  }
}
