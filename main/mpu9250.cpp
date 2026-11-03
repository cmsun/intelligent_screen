#include "mpu9250.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>

#include "esplog.hpp"

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

  // ---- 加速度计校正：只要加速度计有效就参与，不受磁力计开关影响 ----
  // 注意：历史上这一项被错误地放在磁力计有效分支内，导致关闭磁力计融合时
  // 加速度计校正被连带跳过，姿态退化为纯陀螺积分、roll/pitch 随零偏单调漂移。
  float ax = accel.x, ay = accel.y, az = accel.z;
  const float aNorm = std::sqrt(ax * ax + ay * ay + az * az);
  if (aNorm > 1e-6f) {
    ax /= aNorm;
    ay /= aNorm;
    az /= aNorm;

    // 机体系内"估计的重力方向"与实测值之差
    const float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    const float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    const float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    // ∇(½·|f|²) = J_gᵀ·f
    const float g0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
    const float g1 = 2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
    const float g2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
    const float g3 = 2.0f * q1 * f1 + 2.0f * q2 * f2;

    qDot0 -= _beta * g0;
    qDot1 -= _beta * g1;
    qDot2 -= _beta * g2;
    qDot3 -= _beta * g3;
  }

  // ---- 磁力计校正：磁力计无效时跳过，不影响上面的加速度计项 ----
  // 把实测磁场旋到地理系得 h = (hx, hy, hz)，航向误差就是 atan2(-hy, hx)
  // （h 的水平分量应落在地理 +X 上）。直接绕【地理 Z 轴】转过这个角度来修正。
  //
  // 这里不用 Madgwick 的梯度式 f_b：那个目标函数 F = ½|f_b|² = r·(r − hx)
  // 只在"hy = 0"处为零，而 hy = 0 既可以靠 yaw 达成、也可以靠倾斜达成
  // （例如滚转 atan(Bh/Bz) 也能让 hy 归零）。结果是航向误差会耦合进倾斜，
  // 被加速度计项拽住，beta_mag 调大很快就饱和（离线实测：90° 误差
  // beta_mag 0.6→3→6 只从 2.54s 缩到 1.62→1.49s）。
  // 直接修正航向则与倾斜解耦，收敛时间常数就是 1/beta_mag。
  float mx = mag.x, my = mag.y, mz = mag.z;
  const float mNorm = std::sqrt(mx * mx + my * my + mz * mz);
  if (mNorm > 1e-6f) {
    mx /= mNorm;
    my /= mNorm;
    mz /= mNorm;

    // h = Rᵀ(q)·m（机体系 -> 地理系），已按当前姿态做了倾斜补偿
    const float hx = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * mx +
                     2.0f * (q1 * q2 - q0 * q3) * my +
                     2.0f * (q1 * q3 + q0 * q2) * mz;
    const float hy = 2.0f * (q1 * q2 + q0 * q3) * mx +
                     (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * my +
                     2.0f * (q2 * q3 - q0 * q1) * mz;
    const float r = std::sqrt(hx * hx + hy * hy); // 水平分量强度
    if (r > 1e-3f) {
      // yawErr > 0 表示估计航向需要再转 +yawErr 才与磁北一致
      const float yawErr = std::atan2(-hy, hx);
      const float w = 0.5f * _beta_mag * yawErr; // ½·角速度 [rad/s]
      // 航向误差是地理系 Z 轴上的量，须【左乘】（地理系旋转）：
      // qDot += (0,0,0,w) ⊗ q = w·(-q3, -q2, q1, q0)
      qDot0 -= w * q3;
      qDot1 -= w * q2;
      qDot2 += w * q1;
      qDot3 += w * q0;
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
  _fusion.set_beta_mag(_cfg.beta_mag);

  if (auto err = init(); err != ESP_OK) {
    esplog::error("MPU-9250 init failed: {}", esp_err_to_name(err));
    return;
  }
  esplog::info("MPU-9250 initialized");

  // 陀螺仪零偏自动标定：上电后等板子静止即采样求均值。
  // 标定完成前姿态仍会用旧零偏积分，标定结束后会 reset 融合器重新收敛。
  if (_cfg.gyro_auto_calib_enabled) {
    request_gyro_calibration();
  } else {
    _gyro_calib.done = true; // 不标定，直接使用配置里的零偏
  }

  // 磁力计硬磁标定需要人工翻动板子，默认只在显式开启时才上电自动跑
  if (_cfg.mag_calib_on_boot) {
    start_mag_calibration();
  }

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

    ImuSample s;
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
                    "temp={:.2f} | roll={:.1f} pitch={:.1f} yaw={:.1f} "
                    "yaw_rate={:.2f} deg/s",
                    ls.accel.x, ls.accel.y, ls.accel.z, ls.gyro.x, ls.gyro.y,
                    ls.gyro.z, ls.mag.x, ls.mag.y, ls.mag.z, ls.temperature, e.x,
                    e.y, e.z, self->yaw_rate_deg_s());
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

  // 加速度计：原始值 -> 物理单位 -> 尺度 -> 减零偏 -> 转到设备框。
  // 零偏是芯片框下的，必须在乘安装矩阵【之前】减，这样它会被矩阵一起变换。
  const Vec3 accel_chip{
      _cfg.accel_scale.x * to_int16(abuf[0], abuf[1]) * ACCEL_SCALE * G_TO_MS2 -
          _cfg.accel_bias_m_s2.x,
      _cfg.accel_scale.y * to_int16(abuf[2], abuf[3]) * ACCEL_SCALE * G_TO_MS2 -
          _cfg.accel_bias_m_s2.y,
      _cfg.accel_scale.z * to_int16(abuf[4], abuf[5]) * ACCEL_SCALE * G_TO_MS2 -
          _cfg.accel_bias_m_s2.z};
  out.accel = mul(_cfg.chip_to_device, accel_chip);

  // 陀螺仪：同样先在芯片框下减零偏，再转设备框。喂给标定器的是芯片框原始值，
  // 因此自动标定得到的零偏与 gyro_bias_rad_s 处在同一坐标系。
  const Vec3 gyro_chip{
      to_int16(gbuf[0], gbuf[1]) * GYRO_SCALE * RAD_PER_DEG,
      to_int16(gbuf[2], gbuf[3]) * GYRO_SCALE * RAD_PER_DEG,
      to_int16(gbuf[4], gbuf[5]) * GYRO_SCALE * RAD_PER_DEG};
  const float accel_norm = std::sqrt(out.accel.x * out.accel.x +
                                     out.accel.y * out.accel.y +
                                     out.accel.z * out.accel.z);
  gyro_calib_feed(gyro_chip, accel_norm);
  out.gyro = mul(_cfg.chip_to_device,
                 Vec3{gyro_chip.x - _cfg.gyro_bias_rad_s.x,
                      gyro_chip.y - _cfg.gyro_bias_rad_s.y,
                      gyro_chip.z - _cfg.gyro_bias_rad_s.z});

  out.temperature = to_int16(tbuf[0], tbuf[1]) / 333.87f + 21.0f;

  // 磁力计：mbuf[0]=ST1, [1..6]=HXL..HZH, [7]=ST2
  if (mbuf[0] & 0x01) { // DRDY
    Vec3 mraw{};
    mraw.x = to_int16(mbuf[2], mbuf[1]) * AK8963_UT_PER_LSB * _mag_asa[0];
    mraw.y = to_int16(mbuf[4], mbuf[3]) * AK8963_UT_PER_LSB * _mag_asa[1];
    mraw.z = to_int16(mbuf[6], mbuf[5]) * AK8963_UT_PER_LSB * _mag_asa[2];
    // 硬磁标定采集的是「AK8963 原始框、未减偏置」的读数
    mag_calib_feed(mraw, static_cast<uint32_t>(esp_timer_get_time() / 1000));
    // 先在 AK8963 原始框下减硬磁偏置（使读数关于 0 对称），再依次转到芯片框、
    // 设备框 —— 与 accel/gyro「原始框减零偏 -> 乘安装矩阵」的处理保持一致，
    // 这样上面两个矩阵以后怎么改都不必再动 mag_offset_ut。
    _last_mag = mul(_cfg.chip_to_device,
                    mul(_cfg.mag_frame_to_chip,
                        Vec3{mraw.x - _cfg.mag_offset_ut.x,
                             mraw.y - _cfg.mag_offset_ut.y,
                             mraw.z - _cfg.mag_offset_ut.z}));
  }
  out.mag = _last_mag;
  _last_sample = out;
  return ESP_OK;
}

void MPU9250::request_gyro_calibration() noexcept {
  _gyro_calib = GyroCalib{};
  _gyro_calib.started = true;
  _gyro_calib.start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  esplog::info("gyro bias calibration pending, keep the board still...");
}

void MPU9250::start_mag_calibration() noexcept {
  _mag_calib = MagCalib{};
  _mag_calib.active = true;
  _mag_calib.start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  esplog::warn("磁力计硬磁标定开始：请在 {} s 内【持续缓慢翻动板子】，"
               "让 X/Y/Z 三轴都扫到正负两极（画 8 字 + 各方向翻转）",
               static_cast<unsigned>(_cfg.mag_calib_duration_ms / 1000));
  esplog::warn("标定期间磁力计融合会临时关闭，结束后自动恢复");
}

void MPU9250::mag_calib_feed(const Vec3 &mag_raw, uint32_t now_ms) noexcept {
  auto &c = _mag_calib;
  if (!c.active)
    return;

  if (!c.primed) {
    c.min = mag_raw;
    c.max = mag_raw;
    c.primed = true;
  } else {
    c.min.x = std::min(c.min.x, mag_raw.x);
    c.min.y = std::min(c.min.y, mag_raw.y);
    c.min.z = std::min(c.min.z, mag_raw.z);
    c.max.x = std::max(c.max.x, mag_raw.x);
    c.max.y = std::max(c.max.y, mag_raw.y);
    c.max.z = std::max(c.max.z, mag_raw.z);
  }

  if (now_ms - c.start_ms < _cfg.mag_calib_duration_ms)
    return;

  // 采集结束：offset = (max + min) / 2；峰峰值太小的轴视为没转够，保留旧值
  const float range[3] = {c.max.x - c.min.x, c.max.y - c.min.y,
                          c.max.z - c.min.z};
  const float mid[3] = {(c.max.x + c.min.x) * 0.5f, (c.max.y + c.min.y) * 0.5f,
                        (c.max.z + c.min.z) * 0.5f};
  const float old[3] = {_cfg.mag_offset_ut.x, _cfg.mag_offset_ut.y,
                        _cfg.mag_offset_ut.z};
  float next[3];
  for (int i = 0; i < 3; ++i)
    next[i] = (range[i] >= _cfg.mag_calib_min_range_ut) ? mid[i] : old[i];
  _cfg.mag_offset_ut = {next[0], next[1], next[2]};
  c.active = false;
  c.done = true;
  _fusion.reset(); // 航向参考变了，重新起算

  const float radius = (range[0] + range[1] + range[2]) / 6.0f;
  esplog::warn("磁力计硬磁标定完成 -> mag_offset_ut = {{{:.2f}, {:.2f}, {:.2f}}}",
               _cfg.mag_offset_ut.x, _cfg.mag_offset_ut.y,
               _cfg.mag_offset_ut.z);
  esplog::warn("  峰峰值 X={:.1f} Y={:.1f} Z={:.1f} uT（<{:.0f} 的轴沿用旧值）",
               range[0], range[1], range[2], _cfg.mag_calib_min_range_ut);
  esplog::warn("  地磁场强度约 {:.1f} uT（中国大部 45~55）；"
               "把这个 offset 回填到 mpu9250.hpp 后记得把 mag_calib_on_boot 置回 false",
               radius);
}

void MPU9250::gyro_calib_feed(const Vec3 &gyro_raw, float accel_norm) noexcept {
  auto &c = _gyro_calib;
  if (c.done || !_cfg.gyro_auto_calib_enabled)
    return;

  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  if (!c.started) {
    c.started = true;
    c.start_ms = now;
    return;
  }
  // 上电预热：等陀螺/加速度 DLPF 与 PLL 稳定后再开始采样
  if (now - c.start_ms < _cfg.gyro_calib_warmup_ms)
    return;

  // 静止判据 1：三轴角速度都在阈值内
  const bool still = std::fabs(gyro_raw.x) < _cfg.gyro_calib_gyro_thresh &&
                     std::fabs(gyro_raw.y) < _cfg.gyro_calib_gyro_thresh &&
                     std::fabs(gyro_raw.z) < _cfg.gyro_calib_gyro_thresh;
  // 静止判据 2：加速度模值相对本段基准没有明显抖动
  if (!still) {
    c.count = 0;
    c.sum = {};
    return;
  }
  if (c.count == 0) {
    c.accel_ref = accel_norm;
  } else if (std::fabs(accel_norm - c.accel_ref) >
             _cfg.gyro_calib_accel_thresh) {
    c.count = 0;
    c.sum = {};
    return;
  }

  c.sum.x += gyro_raw.x;
  c.sum.y += gyro_raw.y;
  c.sum.z += gyro_raw.z;
  ++c.count;

  if (c.count >= _cfg.gyro_calib_samples) {
    const float inv = 1.0f / static_cast<float>(c.count);
    _cfg.gyro_bias_rad_s = {c.sum.x * inv, c.sum.y * inv, c.sum.z * inv};
    c.done = true;
    // 零偏变了，让姿态从当前姿态重新起算（加速度计会在 ~1/beta 秒内收敛）
    _fusion.reset();
    esplog::info("gyro bias calibrated: {:.5f},{:.5f},{:.5f} rad/s "
                 "({:.4f},{:.4f},{:.4f} deg/s, {} samples)",
                 _cfg.gyro_bias_rad_s.x, _cfg.gyro_bias_rad_s.y,
                 _cfg.gyro_bias_rad_s.z, _cfg.gyro_bias_rad_s.x * 57.29578f,
                 _cfg.gyro_bias_rad_s.y * 57.29578f,
                 _cfg.gyro_bias_rad_s.z * 57.29578f,
                 static_cast<unsigned>(c.count));
  }
}

Vec3 MPU9250::euler_deg() const noexcept {
  Vec3 e = _fusion.euler_deg();
  // 应用静态零偏校准（roll/pitch 基准偏移）
  e.x -= _cfg.roll_offset_deg;
  e.y -= _cfg.pitch_offset_deg;
  // pitch 取反：滤波器内部 pitch 是绕设备 +Y（左）轴的右手角，在设备框
  // （X 前 / Y 左 / Z 上）里 Z→X 即机头下俯，也就是「低头为正」。
  // 常规 / 航空约定是「抬头为正」，所以输出取反。
  // roll 不用动：绕 +X 右手是 Y→Z（左翼抬起 = 右滚），与「右滚为正」一致。
  // 只改显示值，不影响融合器内部四元数；pitch_offset_deg 的填法也不变
  // （显示 = -(raw - offset)，要归零仍是填滤波器原始输出）。
  e.y = -e.y;
  // yaw 取反：滤波器内部 yaw 是绕地理 +Z（向上）的右手角，在设备框
  // （X 前 / Y 左 / Z 上）里即「左转为正」。常规航向角是罗盘角（北 0 / 东 90 /
  // 南 180，右转为正），两者方向相反，所以这里取反后再输出：
  //   设备右转 -> yaw 增大；开启磁力计融合时也就是标准罗盘航向。
  // 实测佐证（2026-11-03）：静止时 gyro.z 残余 +0.02 rad/s，yaw 单调增大，
  // 而 gyro.z > 0 正是左转。若确实需要「左转为正」，去掉下面的负号即可。
  // 归一到 [0, 360)：atan2 原始输出是 (-180, 180]，取两次 fmod 可同时
  // 处理负角与 -0.0（避免出现 "-0.0"）
  e.z = std::fmod(std::fmod(-e.z, 360.0f) + 360.0f, 360.0f);
  return e;
}

esp_err_t MPU9250::read(ImuSample &out, float dt) noexcept {
  esp_err_t err = read_raw(out);
  if (err == ESP_OK && dt > 0.0f) {
    // 三种情况传零磁力计：未启用融合、正在做硬磁标定（避免脏数据污染航向）。
    // update 内只会跳过磁力计校正项，加速度计校正照常生效（两者互不影响）
    const bool use_mag = _cfg.mag_fusion_enabled && !_mag_calib.active;
    const Vec3 mag = use_mag ? out.mag : Vec3{0.0f, 0.0f, 0.0f};
    _fusion.update(out.accel, out.gyro, mag, dt);
  }
  return err;
}
