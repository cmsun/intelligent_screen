// ============================================================================
//  hc_sr04p.h  —  三路 HC-SR04P 超声波测距库（RK3588 / Linux / C++17）
//
//  这个头文件是唯一的公开接口，不依赖 <linux/gpio.h> 等内核头；
//  实现见 hc_sr04p.cpp（建议编成 libhcsr04p.a 后链接）。
//
//  典型用法（其它模块直接 include 即可）：
//
//    #include "hc_sr04p.h"
//
//    hcsr04p::Config cfg;                 // 已预置 左/中/右 三路默认引脚
//    cfg.temp_c = 26.5;                  // 实测气温
//    cfg.sensors[hcsr04p::Right].trig.chip = 4;   // 板上 gpiochip 编号不同时改这里
//    cfg.sensors[hcsr04p::Right].trig.offset = 7;
//
//    hcsr04p::UltrasonicArray us;
//    std::string err;
//    if (!us.open(cfg, {hcsr04p::Left, hcsr04p::Center, hcsr04p::Right}, err)) {
//        // err 里带路名和引脚，直接打日志
//    }
//    hcsr04p::Reading r;
//    if (us.measure(r)) {                // 阻塞一轮，约 samples*interval_ms
//        double left_cm = r[hcsr04p::Left].distance_cm;
//        if (r[hcsr04p::Center].ok()) { ... }
//        if (r[hcsr04p::Right].status == hcsr04p::Status::NoEcho) { ... }
//    }
//    // us.close();  析构会自动释放 GPIO 行
//
//  只需要单路、或想自己控制采样节奏时，用 hcsr04p::Sensor::pulse()；
//  只想做单位换算/统计时，用 widthToCm() / robustWidth() / computeStats()。
//
//  线程安全：单个实例不是线程安全的（GPIO 行有状态），多线程请自行加锁。
//  阻塞：measure() / pulse() 是阻塞调用；可给 measure() 传停止谓词提前退出。
// ============================================================================

#ifndef HC_SR04P_H_
#define HC_SR04P_H_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hcsr04p {

// ---------------------------------------------------------------- 引脚/配置
// RK3588 一个 gpiochip 管 32 条线：offset = 组号(A=0,B=1,C=2,D=3) * 8 + 序号
// 例：GPIO1_D7 = 3*8+7 = 31，GPIO4_A7 = 0*8+7 = 7
struct GpioPin {
    int chip = 0;              // gpiochip 编号，对应 /dev/gpiochipN
    unsigned offset = 0;       // 组内偏移 0..31
    std::string name;          // 仅用于日志，如 "GPIO1_A2"
};

struct SensorConfig {
    std::string name;          // 打印用中文名："左"/"中"/"右"
    std::string key;           // 英文名，用作 GPIO consumer 标签："left"...
    GpioPin trig;
    GpioPin echo;
};

enum SensorIndex : int {
    Left = 0,
    Center = 1,
    Right = 2,
    SensorCount = 3,
};

// 默认三路接线：
//   左  Trig GPIO1_A2 (chip1:2)    Echo GPIO1_A4 (chip1:4)
//   中  Trig GPIO1_A6 (chip1:6)    Echo GPIO1_D7 (chip1:31)
//   右  Trig GPIO4_A7 (chip4:7)    Echo GPIO4_A6 (chip4:6)
// 板子 gpiochip 编号不同时传 chip1 / chip4，或直接改返回的 sensors 数组。
inline std::array<SensorConfig, SensorCount> defaultSensors(int chip1 = 1,
                                                           int chip4 = 4) {
    return {{
        {"左", "left",   {chip1,  2, "GPIO1_A2"}, {chip1,  4, "GPIO1_A4"}},
        {"中", "center", {chip1,  6, "GPIO1_A6"}, {chip1, 31, "GPIO1_D7"}},
        {"右", "right",  {chip4,  7, "GPIO4_A7"}, {chip4,  6, "GPIO4_A6"}},
    }};
}

struct Config {
    std::array<SensorConfig, SensorCount> sensors = defaultSensors();
    int samples = 5;            // 每轮每路采样次数，奇数（中值需要）
    int interval_ms = 80;       // 轮间隔 ms，模块规格要求 >= 60
    int gap_ms = 20;            // 同一轮内相邻两路触发间隔 ms（抑制串扰）
    int timeout_ms = 40;        // 单次回波超时 ms（40 ms 约 6.8 m）
    double temp_c = 25.0;       // 气温，用于声速补偿
    double k = 1.0;             // 标定：真值 = k * 读数 + b
    double b = 0.0;
    bool robust = true;         // true = 中值剔野值后取均值；false = 直接取均值
};

// ---------------------------------------------------------------- 测量结果
enum class Status {
    NotActive,   // 这一路本次没有启用
    Ok,          // 有有效距离
    NoEcho,      // 无有效回波（超量程 / 目标太软 / 盲区）
    Error,       // GPIO 出错，详情见 Measurement::error
};

struct Measurement {
    Status status = Status::NotActive;
    double distance_cm = 0.0;
    uint64_t width_ns = 0;      // 鲁棒统计后的回波高电平宽度
    int valid_samples = 0;      // 拿到回波的次数
    int total_samples = 0;      // 实际采样轮数
    std::string error;          // status == Error 时的原因

    bool ok() const { return status == Status::Ok; }
};

using Reading = std::array<Measurement, SensorCount>;

const char* statusName(Status s);

// ---------------------------------------------------------------- 统计工具
struct Stats {
    int n = 0;
    double median_us = 0.0;
    double mean_us = 0.0;
    double stdev_us = 0.0;
    double min_us = 0.0;
    double max_us = 0.0;
    double p1_us = 0.0;
    double p99_us = 0.0;
};

Stats computeStats(const std::vector<uint64_t>& raw_ns, bool& outlier_removed);

// 「中值 -> 剔除偏离中值 5% 以上的野值 -> 均值」，返回宽度 ns；样本 < 3 返回 0
uint64_t robustWidth(const std::vector<uint64_t>& raw_ns);
uint64_t meanWidth(const std::vector<uint64_t>& raw_ns);

double speedCmUs(double temp_c);
double widthToCm(uint64_t width_ns, double temp_c, double k = 1.0, double b = 0.0);

// ---------------------------------------------------------------- 单路传感器
// 只管一路；一般用 UltrasonicArray，需要自定义调度时才直接用这个类。
class Sensor {
public:
    Sensor();
    ~Sensor();
    Sensor(Sensor&&) noexcept;
    Sensor& operator=(Sensor&&) noexcept;
    Sensor(const Sensor&) = delete;
    Sensor& operator=(const Sensor&) = delete;

    // 申请 Trig 输出行 + Echo 双边沿输入行；失败时 err 带引脚与排查建议
    bool open(const SensorConfig& cfg, int timeout_ms, std::string& err);
    void close();
    bool isOpen() const;
    const SensorConfig& config() const;

    // 触发一次并返回回波高电平宽度（ns）。
    // true  = 拿到有效回波；
    // false + err 为空 = 本次无回波（正常现象，不代表故障）；
    // false + err 非空 = GPIO 出错，应停止使用并重新 open()。
    bool pulse(uint64_t& width_ns, std::string& err);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ---------------------------------------------------------------- 三路管理器
class UltrasonicArray {
public:
    // 返回 true 时提前结束本轮采样（例如 Ctrl+C / 程序退出）
    using StopPredicate = std::function<bool()>;

    UltrasonicArray();
    ~UltrasonicArray();
    UltrasonicArray(UltrasonicArray&&) noexcept;
    UltrasonicArray& operator=(UltrasonicArray&&) noexcept;
    UltrasonicArray(const UltrasonicArray&) = delete;
    UltrasonicArray& operator=(const UltrasonicArray&) = delete;

    // 校验配置并申请 GPIO；active 为要启用的路索引（0/1/2），可只传一路。
    // 任一申请失败会回滚已打开的行，err 说明是哪一路、哪个引脚。
    bool open(const Config& cfg, const std::vector<int>& active, std::string& err);
    bool open(const Config& cfg, std::string& err) {
        return open(cfg, {Left, Center, Right}, err);
    }
    void close();
    bool isOpen() const;

    const Config& config() const;
    const std::vector<int>& active() const;
    const SensorConfig& sensorConfig(int index) const;

    // 一轮完整测量：一轮内逐路触发、等本路回波结束再触发下一路（避免串扰），
    // 共 samples 轮，然后逐路做鲁棒统计与温度/标定换算。
    // 返回 true = 至少一路拿到有效距离；逐路状态见 out。
    bool measure(Reading& out, StopPredicate shouldStop = {});

    // 单次触发某一路，供调用方自定义采样与滤波
    bool pulse(int index, uint64_t& width_ns, std::string& err);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace hcsr04p

#endif  // HC_SR04P_H_
