#include "mpu9250.hpp"

#include <cmath>
#include <cstring>
#include <numbers>

#include "esplog.hpp"

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace imu {
namespace {

// ---- MPU-9250 (MPU-6500 部分) 寄存器 ----
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG_2 = 0x1D;
constexpr uint8_t REG_INT_PIN_CFG = 0x37;
constexpr uint8_t REG_INT_ENABLE = 0x38;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_TEMP_OUT_H = 0x41;
constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
constexpr uint8_t REG_USER_CTRL = 0x6A;
constexpr uint8_t REG_I2C_MST_CTRL = 0x24;
constexpr uint8_t REG_I2C_SLV0_ADDR = 0x25;
constexpr uint8_t REG_I2C_SLV0_REG = 0x26;
constexpr uint8_t REG_I2C_SLV0_CTRL = 0x27;
constexpr uint8_t REG_I2C_SLV0_DO = 0x63;
constexpr uint8_t REG_EXT_SENS_DATA_00 = 0x49;

// MPU-9250 的 WHO_AM_I 为 0x71；其引脚兼容变体 MPU-9255 为 0x73，
// 二者寄存器完全一致，此处同时接受以避免误报初始化失败。
constexpr uint8_t WHO_AM_I_VAL_MPU9250 = 0x71;
constexpr uint8_t WHO_AM_I_VAL_MPU9255 = 0x73;
constexpr uint8_t AK8963_ADDR = 0x0C; // 7-bit I2C 地址

// 量程灵敏度（用于原始值 -> 物理量换算）
constexpr float ACCEL_SCALE = 16.0f / 32768.0f;  // ±16g
constexpr float GYRO_SCALE = 2000.0f / 32768.0f; // ±2000 dps
constexpr float RAD_PER_DEG = std::numbers::pi_v<float> / 180.0f;
constexpr float G_TO_MS2 = 9.80665f;
constexpr float AK8963_UT_PER_LSB = 0.15f; // 16-bit 模式

constexpr int16_t to_int16(uint8_t hi, uint8_t lo) noexcept {
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

} // namespace

// ===================== Madgwick 融合 =====================

Vec3 Madgwick::euler_rad() const noexcept {
  const float &q0 = _q[0];
  const float &q1 = _q[1];
  const float &q2 = _q[2];
  const float &q3 = _q[3];
  Vec3 e;
  e.x = std::atan2(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)); // roll
  const float sinp = 2.0f * (q0 * q2 - q3 * q1);
  e.y = (std::abs(sinp) >= 1.0f)
            ? std::copysign(std::numbers::pi_v<float> / 2.0f, sinp)
            : std::asin(sinp); // pitch
  e.z = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                   1.0f - 2.0f * (q2 * q2 + q3 * q3)); // yaw
  return e;
}

Vec3 Madgwick::euler_deg() const noexcept {
  Vec3 e = euler_rad();
  e.x *= (180.0f / std::numbers::pi_v<float>);
  e.y *= (180.0f / std::numbers::pi_v<float>);
  e.z *= (180.0f / std::numbers::pi_v<float>);
  return e;
}

void Madgwick::update(const Vec3 &accel, const Vec3 &gyro, const Vec3 &mag,
                      float dt) noexcept {
  if (dt <= 0.0f)
    return;

  float q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];

  // 陀螺仪积分（四元数导数）
  float qDot0 = 0.5f * (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z);
  float qDot1 = 0.5f * (q0 * gyro.x + q2 * gyro.z - q3 * gyro.y);
  float qDot2 = 0.5f * (q0 * gyro.y - q1 * gyro.z + q3 * gyro.x);
  float qDot3 = 0.5f * (q0 * gyro.z + q1 * gyro.y - q2 * gyro.x);

  float ax = accel.x, ay = accel.y, az = accel.z;
  float mx = mag.x, my = mag.y, mz = mag.z;

  // 仅在加速度计有效时做梯度校正（避免除零）
  const float aNorm = std::sqrt(ax * ax + ay * ay + az * az);
  if (aNorm > 1e-6f) {
    ax /= aNorm;
    ay /= aNorm;
    az /= aNorm;

    const float mNorm = std::sqrt(mx * mx + my * my + mz * mz);
    if (mNorm > 1e-6f) {
      mx /= mNorm;
      my /= mNorm;
      mz /= mNorm;

      const float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2,
                  _2q3 = 2.0f * q3;
      const float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my,
                  _2q0mz = 2.0f * q0 * mz;
      const float _2q1mx = 2.0f * q1 * mx;
      const float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2,
                  q3q3 = q3 * q3;

      // 地球磁场参考方向
      const float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 +
                       _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      const float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 -
                       my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
      const float _2bx = std::sqrt(hx * hx + hy * hy);
      const float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 -
                         mz * q1q1 + _2q2 * my * q3 - mz * q2q2 - mz * q3q3;
      const float _4bx = 2.0f * _2bx;
      const float _4bz = 2.0f * _2bz;

      // 梯度下降校正步
      float s0 =
          -_2q2 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
          _2q1 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
          _2bz * q2 *
              (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1 * q3 - q0 * q2) - mx) +
          (-_2bx * q3 + _2bz * q1) *
              (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q2 * q3 - q0 * q1) - my) +
          _2bx * q2 *
              (_2bx * (q0 * q2 + q1 * q3) + _2bz * (q1 * q2 - q0 * q3) - mz);

      float s1 =
          _2q3 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
          _2q0 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
          4.0f * q1 * (2.0f * (0.5f - q1q1 - q2q2) - az) +
          _2bz * q3 *
              (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1 * q3 - q0 * q2) - mx) +
          (_2bx * q2 + _2bz * q0) *
              (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q2 * q3 - q0 * q1) - my) +
          (_2bx * q3 - _4bz * q1) *
              (_2bx * (q0 * q2 + q1 * q3) + _2bz * (q1 * q2 - q0 * q3) - mz);

      float s2 =
          -_2q0 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
          _2q3 * (2.0f * (q0 * q1 + q2 * q3) - ay) -
          4.0f * q2 * (2.0f * (0.5f - q1q1 - q2q2) - az) +
          _2bz * q0 *
              (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1 * q3 - q0 * q2) - mx) +
          (-_2bx * q1 + _2bz * q3) *
              (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q2 * q3 - q0 * q1) - my) +
          (_2bx * q0 - _4bz * q2) *
              (_2bx * (q0 * q2 + q1 * q3) + _2bz * (q1 * q2 - q0 * q3) - mz);

      float s3 =
          _2q1 * (2.0f * (q1 * q3 - q0 * q2) - ax) +
          _2q2 * (2.0f * (q0 * q1 + q2 * q3) - ay) +
          (-_4bx * q3 + _2bz * q2) *
              (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1 * q3 - q0 * q2) - mx) +
          (-_2bx * q0 + _2bz * q2) *
              (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q2 * q3 - q0 * q1) - my) +
          (_2bx * q1 + _4bz * q3) *
              (_2bx * (q0 * q2 + q1 * q3) + _2bz * (q1 * q2 - q0 * q3) - mz);

      // 标准 Madgwick：beta 直接乘在原始梯度上，不对 s 归一化。
      // 归一化会破坏梯度下降的物理意义，导致正确解附近持续抖动、漂移。
      qDot0 -= _beta * s0;
      qDot1 -= _beta * s1;
      qDot2 -= _beta * s2;
      qDot3 -= _beta * s3;
    }
  }

  // 积分得到新四元数并归一化
  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;
  const float qNorm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  const float inv = (qNorm > 1e-6f) ? 1.0f / qNorm : 0.0f;
  _q = {q0 * inv, q1 * inv, q2 * inv, q3 * inv};
}

// ===================== MPU9250 驱动 =====================

MPU9250 Imu;

namespace {

// 采样线程栈大小（与 main 中 IMU 线程一致，20K 避免栈溢出）
constexpr uint32_t kImuTaskStackBytes = 20 * 1024;
constexpr UBaseType_t kImuTaskPriority = 5;
constexpr float kLogPeriodS = 0.5f; // 5 Hz 文本日志

} // namespace

MPU9250::~MPU9250() {
  if (_dev != nullptr) {
    spi_bus_remove_device(_dev);
    _dev = nullptr;
  }
  // 不在此释放 SPI 总线（可能在别处复用），如需可调用 spi_bus_free。
}

void MPU9250::begin(void) {
  // 按配置项设置融合增益
  _fusion.set_beta(_cfg.beta);

  if (auto err = init(); err != ESP_OK) {
    esplog::error("MPU-9250 init failed: {}", esp_err_to_name(err));
    return;
  }
  esplog::info("MPU-9250 initialized");

  // 创建采样线程（~200 Hz 读取 + 融合 + 低频日志）
  TaskHandle_t handle = nullptr;
  if (xTaskCreate(sample_task, "imu_sample", kImuTaskStackBytes, this,
                  kImuTaskPriority, &handle) != pdPASS) {
    esplog::error("Failed to create IMU sample task (stack={} bytes)",
                  kImuTaskStackBytes);
    return;
  }
}

void MPU9250::sample_task(void *arg) {
  auto *self = static_cast<MPU9250 *>(arg);

  float last = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
  float last_log = 0.0f;

  while (true) {
    const float now = static_cast<float>(esp_timer_get_time()) / 1.0e6f;
    const float dt = now - last;
    last = now;

    imu::ImuSample s;
    if (self->read(s, dt) != ESP_OK) {
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
      const ImuSample &ls = self->last_sample();
      const Vec3 e = self->euler_deg();
      esplog::debug("accel={:.2f},{:.2f},{:.2f} gyro={:.2f},{:.2f},{:.2f} "
                   "mag={:.2f},{:.2f},{:.2f} "
                   "temp={:.2f} | roll={:.1f} pitch={:.1f} yaw={:.1f}",
                   ls.accel.x, ls.accel.y, ls.accel.z, ls.gyro.x, ls.gyro.y,
                   ls.gyro.z, ls.mag.x, ls.mag.y, ls.mag.z, ls.temperature, e.x,
                   e.y, e.z);
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz 循环
  }
}

esp_err_t MPU9250::write_reg(uint8_t reg, uint8_t val) noexcept {
  if (_dev == nullptr)
    return ESP_FAIL;
  uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};
  spi_transaction_t t{};
  t.length = 16;
  t.tx_buffer = tx;
  return spi_device_transmit(_dev, &t);
}

esp_err_t MPU9250::read_reg(uint8_t reg, uint8_t &val) noexcept {
  if (_dev == nullptr)
    return ESP_FAIL;
  uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0};
  uint8_t rx[2] = {};
  spi_transaction_t t{};
  t.length = 16;
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  esp_err_t err = spi_device_transmit(_dev, &t);
  val = rx[1];
  return err;
}

esp_err_t MPU9250::read_regs(uint8_t reg, uint8_t *out, size_t len) noexcept {
  if (_dev == nullptr)
    return ESP_FAIL;
  if (out == nullptr || len == 0 || len > 31)
    return ESP_ERR_INVALID_ARG;
  std::array<uint8_t, 32> tx{};
  std::array<uint8_t, 32> rx{};
  tx[0] = static_cast<uint8_t>(reg | 0x80);
  spi_transaction_t t{};
  t.length = static_cast<uint32_t>((len + 1) * 8);
  t.tx_buffer = tx.data();
  t.rx_buffer = rx.data();
  esp_err_t err = spi_device_transmit(_dev, &t);
  if (err == ESP_OK)
    std::memcpy(out, rx.data() + 1, len);
  return err;
}

esp_err_t MPU9250::init() noexcept {
  // INT 引脚配置为输入（带上拉），用于数据就绪指示
  gpio_config_t io{};
  io.pin_bit_mask = 1ULL << static_cast<uint32_t>(_cfg.int_pin);
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_ENABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io);

  // 初始化 SPI 总线（不使用 DMA，IMU 传输量小，缓冲用栈内存即可）
  spi_bus_config_t buscfg{};
  buscfg.mosi_io_num = _cfg.sdi;
  buscfg.miso_io_num = _cfg.sdo;
  buscfg.sclk_io_num = _cfg.sck;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 32;
  esp_err_t err = spi_bus_initialize(_cfg.spi_host, &buscfg, SPI_DMA_DISABLED);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    esplog::error("spi_bus_initialize failed: {}", esp_err_to_name(err));
    return err;
  }

  spi_device_interface_config_t devcfg{};
  devcfg.clock_speed_hz = _cfg.spi_clock_hz;
  devcfg.mode = _cfg.spi_mode;
  devcfg.spics_io_num = _cfg.cs;
  devcfg.queue_size = 1;
  devcfg.flags = SPI_DEVICE_NO_DUMMY; // MPU-9250 不插入 dummy 时钟
  err = spi_bus_add_device(_cfg.spi_host, &devcfg, &_dev);
  if (err != ESP_OK) {
    esplog::error("spi_bus_add_device failed: {}", esp_err_to_name(err));
    return err;
  }

  // 复位并唤醒
  write_reg(REG_PWR_MGMT_1, 0x80); // H_RESET
  delay_ms(100);
  write_reg(REG_PWR_MGMT_1, 0x01); // CLKSEL=1 (PLL, gyro X ref)
  delay_ms(10);
  write_reg(REG_PWR_MGMT_2, 0x00); // 使能所有轴

  uint8_t id = 0;
  read_reg(REG_WHO_AM_I, id);
  if (id != WHO_AM_I_VAL_MPU9250 && id != WHO_AM_I_VAL_MPU9255) {
    esplog::error("WHO_AM_I=0x{:02X} (expected 0x71 or 0x73), check wiring/SPI",
                  id);
    return ESP_ERR_NOT_FOUND;
  }

  // 采样率 / 滤波器 / 量程
  write_reg(REG_SMPLRT_DIV, 0x04);   // 1kHz / (1+4) = 200Hz
  write_reg(REG_CONFIG, 0x03);       // DLPF ~44Hz
  write_reg(REG_GYRO_CONFIG, 0x18);  // ±2000 dps
  write_reg(REG_ACCEL_CONFIG, 0x18); // ±16 g
  write_reg(REG_ACCEL_CONFIG_2, 0x03);

  // 数据就绪中断：高电平、推挽
  write_reg(REG_INT_PIN_CFG, 0x00);
  write_reg(REG_INT_ENABLE, 0x01);

  // ---- 通过 MPU 内部 I2C 主设备访问 AK8963 磁力计 ----
  uint8_t u = 0;
  read_reg(REG_USER_CTRL, u);
  write_reg(REG_USER_CTRL, u | 0x20); // I2C_MST_EN
  write_reg(REG_I2C_MST_CTRL, 0x0D);  // I2C 主时钟 ~400kHz

  // 读取 AK8963 灵敏度校正系数 ASAX/Y/Z (0x10..0x12)
  write_reg(REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80); // 读
  write_reg(REG_I2C_SLV0_REG, 0x10);
  write_reg(REG_I2C_SLV0_CTRL, 0x83); // 使能 + 3 字节
  delay_ms(10);
  std::array<uint8_t, 3> asa{};
  read_regs(REG_EXT_SENS_DATA_00, asa.data(), 3);
  for (int i = 0; i < 3; ++i) {
    _mag_asa[i] = (static_cast<float>(asa[i]) - 128.0f) / 256.0f + 1.0f;
  }

  // 配置 AK8963 为连续测量模式2（16-bit, 100Hz）：写 CNTL1=0x16
  write_reg(REG_I2C_SLV0_ADDR, AK8963_ADDR); // 写
  write_reg(REG_I2C_SLV0_REG, 0x0A);         // CNTL1
  write_reg(REG_I2C_SLV0_DO, 0x16);
  write_reg(REG_I2C_SLV0_CTRL, 0x81); // 使能 + 1 字节
  delay_ms(10);

  // 之后每个采样周期由 Slave0 读取 8 字节：ST1 + HXL..HZH + ST2
  write_reg(REG_I2C_SLV0_ADDR, AK8963_ADDR | 0x80);
  write_reg(REG_I2C_SLV0_REG, 0x02);  // ST1
  write_reg(REG_I2C_SLV0_CTRL, 0x88); // 使能 + 8 字节

  _inited = true;
  esplog::info("MPU-9250 initialized (ASA={:.3f},{:.3f},{:.3f})", _mag_asa[0],
               _mag_asa[1], _mag_asa[2]);
  return ESP_OK;
}

esp_err_t MPU9250::read_raw(ImuSample &out) noexcept {
  if (!_inited || _dev == nullptr)
    return ESP_FAIL;

  std::array<uint8_t, 6> abuf{}, gbuf{};
  std::array<uint8_t, 2> tbuf{};
  std::array<uint8_t, 8> mbuf{};

  esp_err_t e1 = read_regs(REG_ACCEL_XOUT_H, abuf.data(), 6);
  esp_err_t e2 = read_regs(REG_GYRO_XOUT_H, gbuf.data(), 6);
  esp_err_t e3 = read_regs(REG_TEMP_OUT_H, tbuf.data(), 2);
  esp_err_t e4 = read_regs(REG_EXT_SENS_DATA_00, mbuf.data(), 8);
  if (e1 != ESP_OK)
    return e1;
  if (e2 != ESP_OK)
    return e2;
  if (e3 != ESP_OK)
    return e3;
  if (e4 != ESP_OK)
    return e4;

  out.accel.x =
      _cfg.accel_sign.x * to_int16(abuf[0], abuf[1]) * ACCEL_SCALE * G_TO_MS2 -
      _cfg.accel_bias_m_s2.x;
  out.accel.y =
      _cfg.accel_sign.y * to_int16(abuf[2], abuf[3]) * ACCEL_SCALE * G_TO_MS2 -
      _cfg.accel_bias_m_s2.y;
  out.accel.z =
      _cfg.accel_sign.z * to_int16(abuf[4], abuf[5]) * ACCEL_SCALE * G_TO_MS2 -
      _cfg.accel_bias_m_s2.z;

  out.gyro.x =
      _cfg.gyro_sign.x * to_int16(gbuf[0], gbuf[1]) * GYRO_SCALE * RAD_PER_DEG -
      _cfg.gyro_bias_rad_s.x;
  out.gyro.y =
      _cfg.gyro_sign.y * to_int16(gbuf[2], gbuf[3]) * GYRO_SCALE * RAD_PER_DEG -
      _cfg.gyro_bias_rad_s.y;
  out.gyro.z =
      _cfg.gyro_sign.z * to_int16(gbuf[4], gbuf[5]) * GYRO_SCALE * RAD_PER_DEG -
      _cfg.gyro_bias_rad_s.z;

  out.temperature = to_int16(tbuf[0], tbuf[1]) / 333.87f + 21.0f;

  // 磁力计：mbuf[0]=ST1, [1..6]=HXL..HZH, [7]=ST2
  if (mbuf[0] & 0x01) { // DRDY
    Vec3 mraw{};
    mraw.x = to_int16(mbuf[2], mbuf[1]) * AK8963_UT_PER_LSB * _mag_asa[0];
    mraw.y = to_int16(mbuf[4], mbuf[3]) * AK8963_UT_PER_LSB * _mag_asa[1];
    mraw.z = to_int16(mbuf[6], mbuf[5]) * AK8963_UT_PER_LSB * _mag_asa[2];
    // 将 AK8963 轴对齐到加速度计坐标系（默认交换 X/Y 并取反 Z）
    Vec3 m = _cfg.mag_swap_xy ? Vec3{mraw.y, mraw.x, -mraw.z} : mraw;
    // 减去硬磁偏置（使读数关于 0 对称），再应用符号校正
    _last_mag = {(m.x - _cfg.mag_offset_ut.x) * _cfg.mag_sign.x,
                 (m.y - _cfg.mag_offset_ut.y) * _cfg.mag_sign.y,
                 (m.z - _cfg.mag_offset_ut.z) * _cfg.mag_sign.z};
  }
  out.mag = _last_mag;
  _last_sample = out;
  return ESP_OK;
}

Vec3 MPU9250::euler_deg() const noexcept {
  Vec3 e = _fusion.euler_deg();
  // 应用静态零偏校准（roll/pitch 基准偏移）
  e.x -= _cfg.roll_offset_deg;
  e.y -= _cfg.pitch_offset_deg;
  return e;
}

esp_err_t MPU9250::read(ImuSample &out, float dt) noexcept {
  esp_err_t err = read_raw(out);
  if (err == ESP_OK && dt > 0.0f) {
    // 未启用磁力计融合时，传入零磁力计（update 内 mNorm<阈值 会跳过磁力计项）
    const Vec3 mag = _cfg.mag_fusion_enabled ? out.mag : Vec3{0.0f, 0.0f, 0.0f};
    _fusion.update(out.accel, out.gyro, mag, dt);
  }
  return err;
}

} // namespace imu
