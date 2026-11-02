#pragma once

#include <array>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
  explicit Madgwick(float beta = 0.1f) noexcept : _beta(beta), _beta_mag(beta) {}

  void set_beta(float beta) noexcept { _beta = beta; }
  /// 磁力计（航向）校正的独立增益，可设得比 beta 大得多
  void set_beta_mag(float beta) noexcept { _beta_mag = beta; }
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

  /// 欧拉角 [deg]：x=roll, y=pitch, z=yaw。yaw 已归一到 [0, 360)。
  [[nodiscard]] Vec3 euler_deg() const noexcept;

private:
  std::array<float, 4> _q{1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
  float _beta;     // 加速度计（重力方向）的梯度下降增益
  float _beta_mag; // 磁力计（航向）的梯度下降增益，独立于 _beta
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

    float beta = 0.6f; // 融合增益（加速度计）

    // 磁力计（航向）校正增益，单位 1/s，航向收敛时间常数 ≈ 1 / beta_mag。
    // 航向用「直接算航向误差 -> 绕地理 Z 轴修正」而不是梯度下降，因此与倾斜
    // 完全解耦：调它不影响 roll/pitch 的收敛速度与抗振性。
    // 离线实测（90° 起始航向误差，板子已水平）：
    //   0.6 -> 3.83s   3.0 -> 0.76s   6.0 -> 0.38s
    // 另：梯度式的有效增益会随「地磁水平分量/总强度」的平方衰减，水平分量弱时
    // 要 13s 以上；直接修正式不受此影响，实测各种倾角下均为 0.76s。
    // 过大（>10）会把磁力计噪声更快带进航向，且收敛受"倾斜先收敛"限制，
    // 收益很快饱和，建议 3~6。
    float beta_mag = 3.0f;

    // 各轴符号校正（与板卡安装 / 芯片朝向相关，按需翻转 ±1）
    Vec3 accel_sign{1.0f, 1.0f, -1.0f};
    Vec3 gyro_sign{1.0f, 1.0f, -1.0f};
    Vec3 mag_sign{1.0f, 1.0f, 1.0f};

    // 加速度计尺度校准（各轴灵敏度修正系数，1.0 = 不修正）。在符号之后、
    // 减零偏之前相乘：out = accel_scale * accel_sign * raw - accel_bias。
    // 实测静止时芯片原始值 |v| = 10.3783 m/s² = 1.05829 g，整体偏大 5.83%。
    // 取 9.80665 / 10.3717 = 0.9455：分母用【z 分量】而非 |v|，因为本配置把
    // X/Y 当作零偏扣掉了（见 accel_bias_m_s2），扣除后垂直分量才应等于 g。
    //   - 若把 X/Y 当真实倾角（bias=0），应改用 |v| 算的 0.9449。
    // 复核：摆几个不同姿态，若 |a| 不再恒为 9.807，说明三轴尺度不一致，需逐轴填。
    Vec3 accel_scale{0.9455f, 0.9455f, 0.9455f};
    bool mag_swap_xy = true; // AK8963 在 MPU-9250 内相对加速度计旋转了约 90°

    // 磁力计硬磁（硬铁）偏置校准，单位：uT。三轴翻转采集最大最小值后计算
    // offset = (max + min) / 2，减去后使读数关于 0 对称。
    // ⚠ 现有的 {18.0, -26.5, 0.0} 是只做水平旋转标定的旧值，z 从没扫到正负两极
    // （所以磁倾角算出来 62°~77°，正常应 35°~50°）。
    // 2026-11-02 三轴翻滚标定结果：{27.96, 10.35, 27.14}，
    // 峰峰值 X=69.4 Y=62.5 Z=76.7 uT，反演地磁场强度约 34.8 uT。
    // ⚠ 该次标定质量存疑：|B| 偏低（中国大部应 45~55），且三轴峰峰值相差 23%
    // （理想应相等）。平放时 |mag|≈31.8 uT、磁倾角≈30.7°。需要远离金属/磁源
    // 重做，见 mag_calib_duration_ms 说明。
    Vec3 mag_offset_ut{27.96f, 10.35f, 27.14f};

    // 是否将磁力计纳入姿态融合。开启后 yaw 才有绝对参考，不会随零偏漂移。
    bool mag_fusion_enabled = true;

    // ---- 磁力计硬磁标定（需人工翻动板子，让三轴都扫到正负两极）----
    // 标定进行中会临时关掉磁力计融合，结束后把结果写回 mag_offset_ut 并恢复。
    bool mag_calib_on_boot = false; // 已回填常量，置回 false；要重标就临时改回 true
    // 采集时长，期间需持续缓慢翻动板子。重做时建议改 60000：
    // 上次 30s 只翻出 34.8 uT 的地磁场强度（应为 45~55），说明覆盖不充分。
    uint32_t mag_calib_duration_ms = 30000;
    float mag_calib_min_range_ut = 20.0f; // 峰峰值低于此值视为该轴没转够，保留旧值

    // 陀螺仪零偏校准（单位：rad/s）。静止时测得的 gyro 输出即为零偏，
    // 读取时减去，消除长时间积分导致的 yaw 漂移。
    // 开启 gyro_auto_calib_enabled 后，该值会被自动标定结果覆盖。
    Vec3 gyro_bias_rad_s{-0.035f, -0.005f, -0.005f};

    // ---- 陀螺仪零偏自动标定（上电/被调用后检测到静止即采样求均值）----
    // 标定要求板子保持不动：任一轴角速度超阈值、或加速度模值抖动超阈值，
    // 都会丢弃已累积的样本重新计数，因此板子动来动去时永远不会标定成功。
    bool gyro_auto_calib_enabled = false;
    uint32_t gyro_calib_samples = 400;     // 采样点数（200Hz 下约 2s）
    uint32_t gyro_calib_warmup_ms = 1000;  // 上电后等待传感器/滤波器稳定
    float gyro_calib_gyro_thresh = 0.05f;  // rad/s，约 2.9°/s
    float gyro_calib_accel_thresh = 0.30f; // m/s²


    // 加速度计零偏校准（单位：m/s^2）。静止水平放置时，理想输出应为
    // (0, 0, ±g)，实测的 X/Y 非零值即为零偏，读取时减去。
    // 2026-11-02 实测：静止时芯片原始值为 (0.0096, 0.3696, -10.3717)。
    // ⚠ 当前是【以现有桌面作为水平基准】的权宜取值：那个桌面实测有约 2.04°
    // 倾角，但暂时找不到更平的水平参考面，于是把 X/Y 分量整体当作零偏扣掉，
    // 使板子放在这张桌子上时输出 (0, 0, +g)。
    // 后果：绝对水平基准被偏移了约 2.04° —— 姿态读数是相对这张桌子的，
    // 不是相对真实水平面。找到水平参考面后需重标：用翻转法
    // (bias = (A + 180°翻转后的读数) / 2) 分离真实零偏与桌面倾角，
    // 并把 accel_scale 相应改为按 |v| 算的 0.9449。
    // z 取 0：单姿态无法分离 z 零偏与尺度误差，这里全部由 accel_scale 承担。
    Vec3 accel_bias_m_s2{0.0091f, 0.3495f, 0.0f};

    // 静态零偏校准（静止水平放置时的输出基准偏移，单位：度），读取时减去。
    // 归零：滤波器修好后平放在参考桌面上输出已接近 (0, 0)，旧的 -27.4 / -12.1
    // 是姿态解算失效（纯陀螺积分）期间标出来的，属于脏数据。
    // 若以后换到真正的水平面，或重新标定了 accel_bias_m_s2，这里需按新的
    // 平放读数重新填（填滤波器原始输出即可使显示归零）。
    float roll_offset_deg = 0.0f;
    float pitch_offset_deg = 0.0f;
  };

  MPU9250() = default;
  ~MPU9250();

  MPU9250(const MPU9250 &) = delete;
  MPU9250 &operator=(const MPU9250 &) = delete;

  /// 初始化 SPI 总线、MPU-9250 及内部 AK8963 磁力计，并创建采样线程。
  /// 与 MotionClass::begin 风格一致：调用后才真正初始化硬件并启动采样。
  void begin(void);

  /// 初始化完成且采样线程已启动。
  [[nodiscard]] bool is_ready() const noexcept { return _inited; }

  /// 重新触发一次陀螺仪零偏标定：丢弃已累积样本，在下一次检测到静止后重新采集。
  /// 调用后请把板子放稳不动，日志会打印标定结果。
  void request_gyro_calibration() noexcept;

  /// 陀螺仪零偏标定是否已完成（false 表示还在等静止或正在采集）。
  [[nodiscard]] bool gyro_calibrated() const noexcept {
    return _gyro_calib.done;
  }

  /// 当前生效的陀螺仪零偏 [rad/s]（自动标定完成后会被更新）。
  [[nodiscard]] Vec3 gyro_bias_rad_s() const noexcept {
    return _cfg.gyro_bias_rad_s;
  }


  /// 配置项（可在 begin 之前修改，begin 之后不再生效）。
  Config _cfg; // 保持与 motion 风格一致，直接公开可改

  /// 最近一次成功读取的原始样本（采样线程持续更新）。
  [[nodiscard]] const ImuSample &last_sample() const noexcept { return _last_sample; }

  [[nodiscard]] std::array<float, 4> quaternion() const noexcept {
    return _fusion.quaternion();
  }
  [[nodiscard]] Vec3 euler_rad() const noexcept { return _fusion.euler_rad(); }
  [[nodiscard]] Vec3 euler_deg() const noexcept;

private:
  // 底层 SPI 辅助函数：init 序列中会忽略返回值，故不加 [[nodiscard]]
  esp_err_t write_reg(uint8_t reg, uint8_t val) noexcept;
  esp_err_t read_reg(uint8_t reg, uint8_t &val) noexcept;
  esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t len) noexcept;
  static void delay_ms(uint32_t ms) noexcept { vTaskDelay(pdMS_TO_TICKS(ms)); }

  /// 初始化 SPI 总线、MPU-9250 及内部 AK8963 磁力计（硬件层）。
  [[nodiscard]] esp_err_t init() noexcept;

  /// 读取并转换 9 轴原始物理量（不更新融合）。
  [[nodiscard]] esp_err_t read_raw(ImuSample &out) noexcept;

  /// 读取传感器并立即执行一次融合；dt 为距上次更新的时间间隔 [s]。
  [[nodiscard]] esp_err_t read(ImuSample &out, float dt) noexcept;

  /// 静止检测 + 陀螺仪零偏累积，由 read_raw 每个采样周期调用。
  void gyro_calib_feed(const Vec3 &gyro_raw, float accel_norm) noexcept;

  /// 磁力计硬磁标定：开始采集 min/max，需人工翻动板子让三轴都扫到正负两极。
  /// 结束后自动把 offset 写回 mag_offset_ut 并恢复磁力计融合。
  void start_mag_calibration() noexcept;

  /// 磁力计硬磁标定是否已完成。
  [[nodiscard]] bool mag_calibrated() const noexcept { return _mag_calib.done; }

  /// 磁力计硬磁标定状态机，由 read_raw 在磁力计数据更新（DRDY）时调用。
  void mag_calib_feed(const Vec3 &mag_aligned, uint32_t now_ms) noexcept;

  /// 采样线程：高频读取 + 融合 + 低频日志。
  static void sample_task(void *arg);

  struct GyroCalib {
    bool started = false;   // 已记录起始时刻（用于上电预热）
    bool done = false;      // 标定完成
    uint32_t count = 0;     // 本段已累计样本数
    Vec3 sum{};             // 原始陀螺累加值
    float accel_ref = 0.0f; // 本段加速度模值基准（用于抖动判定）
    uint32_t start_ms = 0;  // warmup 起点
  };
  GyroCalib _gyro_calib{};

  struct MagCalib {
    bool active = false;   // 正在采集
    bool done = false;     // 标定完成
    bool primed = false;   // 已收到第一个样本
    uint32_t start_ms = 0; // 采集起点
    Vec3 min{};            // 已对齐到加速度计坐标系、未减偏置的磁力计最小值
    Vec3 max{};
  };
  MagCalib _mag_calib{};

  spi_device_handle_t _dev = nullptr;
  Madgwick _fusion;
  std::array<float, 3> _mag_asa{1.0f, 1.0f, 1.0f}; // 磁力计灵敏度校正系数
  Vec3 _last_mag{}; // 最近一次有效磁力计读数（DRDY 前复用）
  ImuSample _last_sample{}; // 最近一次成功读取的完整样本
  bool _inited = false;
};

/// 全局 IMU 实例
extern MPU9250 Imu;
