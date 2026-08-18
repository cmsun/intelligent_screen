#pragma once

#include <array>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace imu {

/// 三维向量（物理单位见各使用处）
struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/// 一次采样的 9 轴原始物理量
struct ImuSample {
  Vec3 accel{};        // [m/s^2]
  Vec3 gyro{};         // [rad/s]
  Vec3 mag{};          // [uT]（微特斯拉）
  float temperature{}; // [°C]
};

/// Madgwick 9 轴（加速度计 + 陀螺仪 + 磁力计）姿态融合滤波器。
/// 输出单位四元数 (w, x, y, z)，可转换为欧拉角（roll/pitch/yaw）。
class Madgwick {
public:
  explicit Madgwick(float beta = 0.1f) noexcept : _beta(beta) {}

  void set_beta(float beta) noexcept { _beta = beta; }
  void reset() noexcept { _q = {1.0f, 0.0f, 0.0f, 0.0f}; }

  /// @param accel 加速度计，任意一致单位（内部会归一化）
  /// @param gyro  陀螺仪，[rad/s]
  /// @param mag   磁力计，任意一致单位（内部会归一化）
  /// @param dt    距上次更新的时间间隔 [s]
  void update(const Vec3 &accel, const Vec3 &gyro, const Vec3 &mag,
              float dt) noexcept;

  /// 单位四元数 (w, x, y, z)
  [[nodiscard]] std::array<float, 4> quaternion() const noexcept { return _q; }

  /// 欧拉角 [rad]：x=roll, y=pitch, z=yaw
  [[nodiscard]] Vec3 euler_rad() const noexcept;

  /// 欧拉角 [deg]：x=roll, y=pitch, z=yaw
  [[nodiscard]] Vec3 euler_deg() const noexcept;

private:
  std::array<float, 4> _q{1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
  float _beta; // 梯度下降收敛增益（越大跟随加速度/磁力越快，但噪声更敏感）
};

/// 通过 SPI 读取 MPU-9250（含内部 AK8963 磁力计）并实现传感器融合。
///
/// 接线（ESP32-S3 侧）：
///   GPIO6 -> MPU SDO (MISO)
///   GPIO7 -> MPU SDI (MOSI)
///   GPIO8 -> MPU SCK
///   GPIO9 -> MPU CS
///   GPIO10-> MPU INT
///
/// 注意：ESP32-S3 的 GPIO6~11 默认被内部 SPI0/1（Flash/PSRAM）占用，
/// 直接用作普通 SPI 需确保硬件上 Flash 已移至其它引脚或已禁用 PSRAM，
/// 否则需修改 sdkconfig（CONFIG_SPIRAM / 相关 Flash 引脚配置）。
class MPU9250 {
public:
  struct Config {
    spi_host_device_t spi_host = SPI2_HOST;
    gpio_num_t sdo = GPIO_NUM_6;       // MISO
    gpio_num_t sdi = GPIO_NUM_7;       // MOSI
    gpio_num_t sck = GPIO_NUM_8;       // SCLK
    gpio_num_t cs = GPIO_NUM_9;        // CS
    gpio_num_t int_pin = GPIO_NUM_10;  // INT（数据就绪，可选）
    uint32_t spi_clock_hz = 1'000'000; // 1 MHz
    uint8_t spi_mode = 3;              // MPU-9250 支持 SPI mode 0 与 3

    float beta = 0.6f; // 融合增益

    // 各轴符号校正（与板卡安装 / 芯片朝向相关，按需翻转 ±1）
    Vec3 accel_sign{1.0f, 1.0f, -1.0f};
    Vec3 gyro_sign{1.0f, 1.0f, -1.0f};
    Vec3 mag_sign{1.0f, 1.0f, 1.0f};
    bool mag_swap_xy = true; // AK8963 在 MPU-9250 内相对加速度计旋转了约 90°

    // 磁力计硬磁偏置校准（单位：uT）。水平旋转采集最大最小值后计算：
    // offset = (max + min) / 2，减去后使读数关于 0 对称。
    // 实测：mag.x 范围 -12~48 -> offset=18；mag.y 范围 -60~7 -> offset=-26.5
    Vec3 mag_offset_ut{18.0f, -26.5f, 0.0f};

    // 是否将磁力计纳入姿态融合（默认不融合，仅用加速度计+陀螺仪）
    bool mag_fusion_enabled = false;

    // 陀螺仪零偏校准（单位：rad/s）。静止时测得的 gyro 输出即为零偏，
    // 读取时减去，消除长时间积分导致的 yaw 漂移。
    Vec3 gyro_bias_rad_s{-0.035f, -0.005f, -0.005f};

    // 加速度计零偏校准（单位：m/s^2）。静止水平放置时，理想输出应为
    // (0, 0, ±g)，实测的 X/Y 非零值即为零偏，读取时减去。
    Vec3 accel_bias_m_s2{-0.30f, 0.08f, -0.25f};

    // 静态零偏校准（静止水平放置时的输出基准偏移，单位：度）
    float roll_offset_deg = -27.4f;
    float pitch_offset_deg = -12.1f;
  };

  explicit MPU9250(const Config &cfg) noexcept : _cfg(cfg), _fusion(cfg.beta) {}
  ~MPU9250();

  MPU9250(const MPU9250 &) = delete;
  MPU9250 &operator=(const MPU9250 &) = delete;

  /// 初始化 SPI 总线、MPU-9250 及内部 AK8963 磁力计。
  [[nodiscard]] esp_err_t init() noexcept;

  /// INT 引脚当前电平（高表示数据就绪，取决于 REG_INT_PIN_CFG 配置）。
  [[nodiscard]] bool is_data_ready() const noexcept;

  /// 读取并转换 9 轴原始物理量（不更新融合）。
  [[nodiscard]] esp_err_t read_raw(ImuSample &out) noexcept;

  /// 读取传感器并立即执行一次融合；dt 为距上次更新的时间间隔 [s]。
  [[nodiscard]] esp_err_t read(ImuSample &out, float dt) noexcept;

  [[nodiscard]] std::array<float, 4> quaternion() const noexcept {
    return _fusion.quaternion();
  }
  [[nodiscard]] Vec3 euler_rad() const noexcept { return _fusion.euler_rad(); }
  [[nodiscard]] Vec3 euler_deg() const noexcept;

  /// 最近一次成功读取的原始样本（read/read_raw 之后有效）。
  [[nodiscard]] const ImuSample &last_sample() const noexcept { return _last_sample; }

private:
  // 底层 SPI 辅助函数：init 序列中会忽略返回值，故不加 [[nodiscard]]
  esp_err_t write_reg(uint8_t reg, uint8_t val) noexcept;
  esp_err_t read_reg(uint8_t reg, uint8_t &val) noexcept;
  esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t len) noexcept;
  static void delay_ms(uint32_t ms) noexcept { vTaskDelay(pdMS_TO_TICKS(ms)); }

  Config _cfg;
  spi_device_handle_t _dev = nullptr;
  Madgwick _fusion;
  std::array<float, 3> _mag_asa{1.0f, 1.0f, 1.0f}; // 磁力计灵敏度校正系数
  Vec3 _last_mag{}; // 最近一次有效磁力计读数（DRDY 前复用）
  ImuSample _last_sample{}; // 最近一次成功读取的完整样本
  bool _inited = false;
};

} // namespace imu
