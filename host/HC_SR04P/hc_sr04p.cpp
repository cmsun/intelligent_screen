// ============================================================================
//  hc_sr04p.cpp  —  三路 HC-SR04P 测距库实现（不含 main，供其它模块链接）
//
//  实现要点
//    直接使用内核 GPIO 字符设备 ABI（linux/gpio.h, ABI v2），不链接 libgpiod，
//    因而 Ubuntu 22.04（libgpiod 1.6）与 24.04（libgpiod 2.x）都能用，内核 >= 5.10。
//    GPIO 细节藏在 detail / pimpl 里，公开头文件不暴露任何内核头。
//    测时读边沿事件的 ev.timestamp_ns（内核在中断中用 CLOCK_MONOTONIC 采集），
//    两个沿同源，采集延迟相互抵消，抖动远小于用户态轮询电平。
//
//  接线与权限说明见 hc_sr04p.h 顶部注释；命令行演示见 hc_sr04p_main.cpp。
// ============================================================================

#include "hc_sr04p.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/gpio.h>

namespace hcsr04p {
namespace detail {

constexpr double kSoundA = 331.3;       // 0 °C 时的声速 m/s
constexpr double kSoundB = 0.606;       // 每升高 1 °C 增加 m/s
constexpr uint64_t kTriggerNs = 12000;  // Trig 高电平，规格只要求 >= 10µs
constexpr double kOutlierRatio = 0.05;  // 偏离中值 5% 以上判为野值

inline uint64_t nowNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull +
           static_cast<uint64_t>(ts.tv_nsec);
}

std::string errnoStr() { return std::string(std::strerror(errno)); }

// 分片睡眠，便于 measure() 在长等待中响应停止谓词
void sleepMs(int ms, const std::function<bool()>& stop = {}) {
    if (ms <= 0) return;
    const uint64_t deadline = nowNs() + static_cast<uint64_t>(ms) * 1000000ull;
    while (nowNs() < deadline) {
        if (stop && stop()) return;
        const uint64_t remain_ns = deadline - nowNs();
        int chunk_ms = static_cast<int>((remain_ns + 999999ull) / 1000000ull);
        if (chunk_ms > 10) chunk_ms = 10;
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = static_cast<long>(chunk_ms) * 1000000L;
        while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}
    }
}

// ---------------------------------------------------------------- GPIO 行
class GpioLine {
public:
    GpioLine() = default;
    ~GpioLine() { close(); }
    GpioLine(const GpioLine&) = delete;
    GpioLine& operator=(const GpioLine&) = delete;

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool isOpen() const { return fd_ >= 0; }

    bool openOutput(int chip_num, unsigned offset, unsigned long long initial,
                    const char* consumer, std::string& err) {
        const int chip_fd = openChip(chip_num, err);
        if (chip_fd < 0) return false;

        struct gpio_v2_line_request req;
        std::memset(&req, 0, sizeof(req));
        req.offsets[0] = offset;
        req.num_lines = 1;
        std::snprintf(req.consumer, sizeof(req.consumer), "%s", consumer);
        req.config.flags = static_cast<unsigned long long>(GPIO_V2_LINE_FLAG_OUTPUT);
        req.config.num_attrs = 1;
        req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
        req.config.attrs[0].attr.values = initial;
        req.config.attrs[0].mask = 1;

        const int rc = ::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
        const std::string saved = errnoStr();
        ::close(chip_fd);
        if (rc < 0) {
            err = "申请输出行失败（gpiochip" + std::to_string(chip_num) + " 偏移 " +
                  std::to_string(offset) + "）: " + saved +
                  "\n  排查：内核是否 >= 5.10？该引脚是否被别的驱动占用或复用了外设功能？"
                  "\n  cat /sys/kernel/debug/gpio ； "
                  "cat /sys/kernel/debug/pinctrl/pinctrl-rockchip-pinctrl/pinmux-pins";
            return false;
        }
        fd_ = req.fd;
        setNonBlock();
        return true;
    }

    bool openInputBothEdges(int chip_num, unsigned offset, const char* consumer,
                            std::string& err) {
        const int chip_fd = openChip(chip_num, err);
        if (chip_fd < 0) return false;

        struct gpio_v2_line_request req;
        std::memset(&req, 0, sizeof(req));
        req.offsets[0] = offset;
        req.num_lines = 1;
        req.event_buffer_size = 16;
        std::snprintf(req.consumer, sizeof(req.consumer), "%s", consumer);
        req.config.flags = static_cast<unsigned long long>(
            GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING |
            GPIO_V2_LINE_FLAG_EDGE_FALLING | GPIO_V2_LINE_FLAG_BIAS_DISABLED);
        req.config.num_attrs = 0;

        const int rc = ::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
        const std::string saved = errnoStr();
        ::close(chip_fd);
        if (rc < 0) {
            err = "申请输入行失败（gpiochip" + std::to_string(chip_num) + " 偏移 " +
                  std::to_string(offset) + "）: " + saved +
                  "\n  排查：同上；另外确认没有别的进程（比如上次没退出的实例）已占用该行。";
            return false;
        }
        fd_ = req.fd;
        setNonBlock();
        return true;
    }

    bool set(bool value, std::string& err) {
        struct gpio_v2_line_values v;
        std::memset(&v, 0, sizeof(v));
        v.mask = 1;
        v.bits = value ? 1ull : 0ull;
        if (::ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) < 0) {
            err = "写 GPIO 失败: " + errnoStr();
            return false;
        }
        return true;
    }

    // 1 = 高电平，0 = 低电平，-1 = 出错
    int getValue(std::string& err) {
        struct gpio_v2_line_values v;
        std::memset(&v, 0, sizeof(v));
        v.mask = 1;
        if (::ioctl(fd_, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0) {
            err = "读 GPIO 失败: " + errnoStr();
            return -1;
        }
        return (v.bits & 1ull) ? 1 : 0;
    }

    // 1 = 有事件可读，0 = 超时，-1 = 出错
    int waitEvent(int timeout_ms) {
        struct pollfd p;
        p.fd = fd_;
        p.events = POLLIN;
        p.revents = 0;
        const uint64_t deadline =
            nowNs() + static_cast<uint64_t>(timeout_ms) * 1000000ull + 1000000ull;
        int r;
        do {
            r = ::poll(&p, 1, timeout_ms);
        } while (r < 0 && errno == EINTR && nowNs() < deadline);
        if (r < 0) return -1;
        if (r == 0) return 0;
        return (p.revents & POLLIN) ? 1 : 0;
    }

    // 1 = 读到事件，0 = 暂无，-1 = 出错
    int readEvent(uint64_t& ts_ns, uint32_t& event_id) {
        struct gpio_v2_line_event ev;
        const ssize_t n = ::read(fd_, &ev, sizeof(ev));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
        if (n != static_cast<ssize_t>(sizeof(ev))) return -1;
        ts_ns = static_cast<uint64_t>(ev.timestamp_ns);
        event_id = ev.id;
        return 1;
    }

    void drain() {
        uint64_t ts = 0;
        uint32_t id = 0;
        while (waitEvent(0) == 1) {
            if (readEvent(ts, id) != 1) break;
        }
    }

private:
    static int openChip(int chip_num, std::string& err) {
        const std::string dev = "/dev/gpiochip" + std::to_string(chip_num);
        const int fd = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) {
            err = "打不开 " + dev + ": " + errnoStr() +
                  "\n  用 gpiodetect 确认 gpiochip 编号，或修改 Config::sensors 里的 chip 字段。";
        }
        return fd;
    }

    void setNonBlock() {
        const int fl = ::fcntl(fd_, F_GETFL, 0);
        if (fl >= 0) ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);
    }

    int fd_ = -1;
};

// Echo 若仍为高，新的上升沿永远不会到来，触发前必须等它变低
void waitEchoLow(GpioLine& echo) {
    const uint64_t deadline = nowNs() + 200000000ull;   // 最多等 200 ms
    while (nowNs() < deadline) {
        std::string err;
        const int lvl = echo.getValue(err);
        if (lvl == 0) return;
        if (lvl < 0) return;                            // 读不到就交给超时逻辑
        sleepMs(2);
    }
}

bool validateConfig(const Config& c, const std::vector<int>& active,
                    std::string& err) {
    if (active.empty()) {
        err = "active 为空：至少要启用一路传感器";
        return false;
    }
    for (int idx : active) {
        if (idx < 0 || idx >= SensorCount) {
            err = "active 含非法索引 " + std::to_string(idx) + "（只能是 0/1/2）";
            return false;
        }
    }
    if (c.samples < 3 || c.samples % 2 == 0) {
        err = "Config::samples 必须是不小于 3 的奇数（中值滤波需要）";
        return false;
    }
    if (c.interval_ms < 60) {
        err = "Config::interval_ms 必须 >= 60（HC-SR04P 规格要求）";
        return false;
    }
    if (c.gap_ms < 0 || c.timeout_ms < 1) {
        err = "Config::gap_ms 不能为负，timeout_ms 必须 > 0";
        return false;
    }
    for (const SensorConfig& s : c.sensors) {
        for (const GpioPin& p : {s.trig, s.echo}) {
            if (p.chip < 0 || p.offset > 31) {
                err = "引脚 " + p.name + " 配置非法：chip 必须 >= 0，offset 必须在 0..31";
                return false;
            }
        }
    }
    return true;
}

}  // namespace detail

// ---------------------------------------------------------------- 统计工具
double speedCmUs(double temp_c) {
    return (detail::kSoundA + detail::kSoundB * temp_c) / 1e4;
}

double widthToCm(uint64_t width_ns, double temp_c, double k, double b) {
    return k * (static_cast<double>(width_ns) * 1e-3 * speedCmUs(temp_c) / 2.0) + b;
}

const char* statusName(Status s) {
    switch (s) {
        case Status::Ok:        return "Ok";
        case Status::NoEcho:    return "NoEcho";
        case Status::Error:     return "Error";
        case Status::NotActive: return "NotActive";
    }
    return "Unknown";
}

namespace detail {

double percentile(const std::vector<double>& sorted, double p) {
    if (sorted.empty()) return 0.0;
    const double idx = p * (static_cast<double>(sorted.size()) - 1.0);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = (lo + 1 < sorted.size()) ? lo + 1 : lo;
    const double frac = idx - static_cast<double>(lo);
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

}  // namespace detail

Stats computeStats(const std::vector<uint64_t>& raw_ns, bool& outlier_removed) {
    Stats s;
    if (raw_ns.empty()) return s;

    std::vector<double> all;
    all.reserve(raw_ns.size());
    for (uint64_t x : raw_ns) all.push_back(static_cast<double>(x) * 1e-3);

    std::vector<double> sorted = all;
    std::sort(sorted.begin(), sorted.end());
    const double med = detail::percentile(sorted, 0.5);

    s.n = static_cast<int>(sorted.size());
    s.median_us = med;
    s.min_us = sorted.front();
    s.max_us = sorted.back();
    s.p1_us = detail::percentile(sorted, 0.01);
    s.p99_us = detail::percentile(sorted, 0.99);

    std::vector<double> good;
    good.reserve(sorted.size());
    for (double v : all) {
        if (std::fabs(v - med) < detail::kOutlierRatio * med) good.push_back(v);
    }
    outlier_removed = (good.size() != all.size());
    if (good.empty()) good = all;

    double sum = 0.0;
    for (double v : good) sum += v;
    s.mean_us = sum / static_cast<double>(good.size());

    double acc = 0.0;
    for (double v : good) acc += (v - s.mean_us) * (v - s.mean_us);
    s.stdev_us = (good.size() > 1)
                     ? std::sqrt(acc / static_cast<double>(good.size() - 1))
                     : 0.0;
    return s;
}

uint64_t robustWidth(const std::vector<uint64_t>& raw_ns) {
    if (raw_ns.size() < 3) return 0;
    bool unused = false;
    const Stats s = computeStats(raw_ns, unused);
    return static_cast<uint64_t>(s.mean_us * 1e3);
}

uint64_t meanWidth(const std::vector<uint64_t>& raw_ns) {
    if (raw_ns.empty()) return 0;
    uint64_t sum = 0;
    for (uint64_t x : raw_ns) sum += x;
    return sum / raw_ns.size();
}

// ---------------------------------------------------------------- Sensor
struct Sensor::Impl {
    SensorConfig cfg;
    detail::GpioLine trig;
    detail::GpioLine echo;
    uint64_t timeout_ns = 40000000;
    bool opened = false;
};

Sensor::Sensor() : impl_(std::make_unique<Impl>()) {}
Sensor::~Sensor() = default;
Sensor::Sensor(Sensor&&) noexcept = default;
Sensor& Sensor::operator=(Sensor&&) noexcept = default;

const SensorConfig& Sensor::config() const {
    static const SensorConfig empty;
    return impl_ ? impl_->cfg : empty;
}

bool Sensor::isOpen() const { return impl_ && impl_->opened; }

bool Sensor::open(const SensorConfig& cfg, int timeout_ms, std::string& err) {
    close();
    if (!impl_) impl_ = std::make_unique<Impl>();
    if (timeout_ms < 1) {
        err = "timeout_ms 必须 > 0";
        return false;
    }

    Impl& d = *impl_;
    d.cfg = cfg;
    d.timeout_ns = static_cast<uint64_t>(timeout_ms) * 1000000ull;
    const std::string tag = cfg.key.empty() ? std::string("sensor") : cfg.key;
    if (!d.trig.openOutput(cfg.trig.chip, cfg.trig.offset, 0,
                           ("sr04-" + tag + "-trig").c_str(), err)) {
        return false;
    }
    if (!d.echo.openInputBothEdges(cfg.echo.chip, cfg.echo.offset,
                                   ("sr04-" + tag + "-echo").c_str(), err)) {
        d.trig.close();
        return false;
    }
    detail::sleepMs(20);       // 等模块上电稳定（首次测量常不可靠）
    d.echo.drain();
    d.opened = true;
    return true;
}

void Sensor::close() {
    if (!impl_) return;
    impl_->trig.close();
    impl_->echo.close();
    impl_->opened = false;
}

bool Sensor::pulse(uint64_t& width_ns, std::string& err) {
    width_ns = 0;
    err.clear();
    if (!isOpen()) {
        err = "传感器未初始化，请先调用 open()";
        return false;
    }

    Impl& d = *impl_;
    d.echo.drain();                      // 丢掉上一轮的残留事件
    detail::waitEchoLow(d.echo);

    if (!d.trig.set(true, err)) return false;
    const uint64_t t0 = detail::nowNs();
    while (detail::nowNs() - t0 < detail::kTriggerNs) {  // 自旋出 12µs
    }
    if (!d.trig.set(false, err)) return false;

    uint64_t t_rise = 0;
    const uint64_t deadline = detail::nowNs() + d.timeout_ns;
    while (detail::nowNs() < deadline) {
        if (d.echo.waitEvent(20) != 1) continue;
        uint64_t ts = 0;
        uint32_t id = 0;
        while (d.echo.readEvent(ts, id) == 1) {
            if (id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
                t_rise = ts;
            } else if (id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
                if (t_rise != 0 && ts > t_rise) {
                    width_ns = ts - t_rise;   // 两沿相减，采集延迟抵消
                    return true;
                }
                break;
            }
        }
    }
    return false;                        // 无回波：err 保持为空
}

// ---------------------------------------------------------------- UltrasonicArray
struct UltrasonicArray::Impl {
    Config cfg;
    std::vector<int> active;
    std::array<Sensor, SensorCount> sensors;
    bool opened = false;
};

UltrasonicArray::UltrasonicArray() : impl_(std::make_unique<Impl>()) {}
UltrasonicArray::~UltrasonicArray() = default;
UltrasonicArray::UltrasonicArray(UltrasonicArray&&) noexcept = default;
UltrasonicArray& UltrasonicArray::operator=(UltrasonicArray&&) noexcept = default;

bool UltrasonicArray::isOpen() const { return impl_ && impl_->opened; }

const Config& UltrasonicArray::config() const {
    static const Config empty;
    return impl_ ? impl_->cfg : empty;
}

const std::vector<int>& UltrasonicArray::active() const {
    static const std::vector<int> empty;
    return impl_ ? impl_->active : empty;
}

const SensorConfig& UltrasonicArray::sensorConfig(int index) const {
    if (index >= 0 && index < SensorCount && impl_) return impl_->cfg.sensors[index];
    static const SensorConfig empty;
    return empty;
}

bool UltrasonicArray::open(const Config& cfg, const std::vector<int>& active,
                           std::string& err) {
    close();
    if (!impl_) impl_ = std::make_unique<Impl>();

    // 去重并保持调用方给的顺序
    std::vector<int> wanted;
    for (int idx : active) {
        if (std::find(wanted.begin(), wanted.end(), idx) == wanted.end()) {
            wanted.push_back(idx);
        }
    }
    if (!detail::validateConfig(cfg, wanted, err)) return false;

    Impl& d = *impl_;
    d.cfg = cfg;
    d.active = wanted;
    for (int idx : wanted) {
        const SensorConfig& sc = cfg.sensors[idx];
        std::string inner;
        if (!d.sensors[idx].open(sc, cfg.timeout_ms, inner)) {
            err = "初始化失败 [" + sc.name + "]：Trig " + sc.trig.name +
                  "=gpiochip" + std::to_string(sc.trig.chip) + ":" +
                  std::to_string(sc.trig.offset) + "，Echo " + sc.echo.name +
                  "=gpiochip" + std::to_string(sc.echo.chip) + ":" +
                  std::to_string(sc.echo.offset) + "\n  " + inner;
            close();                       // 回滚已打开的行
            return false;
        }
    }
    d.opened = true;
    return true;
}

void UltrasonicArray::close() {
    if (!impl_) return;
    for (Sensor& s : impl_->sensors) s.close();
    impl_->active.clear();
    impl_->opened = false;
}

bool UltrasonicArray::pulse(int index, uint64_t& width_ns, std::string& err) {
    width_ns = 0;
    err.clear();
    if (!isOpen()) {
        err = "设备未初始化，请先调用 open()";
        return false;
    }
    if (index < 0 || index >= SensorCount ||
        std::find(impl_->active.begin(), impl_->active.end(), index) ==
            impl_->active.end()) {
        err = "传感器索引 " + std::to_string(index) + " 未在 active 中启用";
        return false;
    }
    return impl_->sensors[index].pulse(width_ns, err);
}

bool UltrasonicArray::measure(Reading& out, StopPredicate shouldStop) {
    for (Measurement& m : out) m = Measurement{};   // 默认 NotActive
    if (!isOpen()) return false;

    Impl& d = *impl_;
    const Config& c = d.cfg;
    std::array<std::vector<uint64_t>, SensorCount> raw;
    std::array<std::string, SensorCount> errors;
    for (int idx : d.active) out[idx].status = Status::NoEcho;

    int rounds = 0;
    for (int i = 0; i < c.samples; ++i) {
        if (shouldStop && shouldStop()) break;
        for (size_t j = 0; j < d.active.size(); ++j) {
            const int idx = d.active[j];
            uint64_t w = 0;
            std::string err;
            if (d.sensors[idx].pulse(w, err)) {
                raw[idx].push_back(w);
            } else if (!err.empty() && errors[idx].empty()) {
                errors[idx] = err;
            }
            if (j + 1 < d.active.size()) detail::sleepMs(c.gap_ms, shouldStop);
        }
        ++rounds;
        if (i + 1 < c.samples) detail::sleepMs(c.interval_ms, shouldStop);
        if (shouldStop && shouldStop()) break;
    }

    bool any_ok = false;
    for (int idx : d.active) {
        Measurement& m = out[idx];
        m.valid_samples = static_cast<int>(raw[idx].size());
        m.total_samples = rounds;
        if (!errors[idx].empty()) {
            m.status = Status::Error;
            m.error = errors[idx];
            continue;
        }
        const uint64_t width = c.robust ? robustWidth(raw[idx]) : meanWidth(raw[idx]);
        if (width == 0) {
            m.status = Status::NoEcho;
            continue;
        }
        m.width_ns = width;
        m.distance_cm = widthToCm(width, c.temp_c, c.k, c.b);
        m.status = Status::Ok;
        any_ok = true;
    }
    return any_ok;
}

}  // namespace hcsr04p
