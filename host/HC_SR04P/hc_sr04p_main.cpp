// ============================================================================
//  hc_sr04p_main.cpp  —  命令行演示程序（不是库的一部分）
//
//  这个文件只负责参数解析、打印、抖动诊断和标定，测距逻辑全部来自
//  hc_sr04p.h / hc_sr04p.cpp。其它模块要引用测距功能时，包含 hc_sr04p.h
//  并链接 libhcsr04p.a 即可，不需要也不应该编译本文件。
//
//  接线
//    左  Trig GPIO1_A2 (gpiochip1:2)    Echo GPIO1_A4 (gpiochip1:4)
//    中  Trig GPIO1_A6 (gpiochip1:6)    Echo GPIO1_D7 (gpiochip1:31)
//    右  Trig GPIO4_A7 (gpiochip4:7)    Echo GPIO4_A6 (gpiochip4:6)
//    VCC 推荐 3.3V；5V 供电时 Echo 必须 1kΩ+2kΩ 分压（GPIO 不耐 5V）；三路共地。
//
//  跑之前核准
//    sudo apt install gpiod -y
//    gpiodetect ; gpiofind "GPIO1_A2" ; gpioinfo gpiochip1
//    编号不同用 --chip1 / --chip4；引脚被复用查 pinmux-pins。
//
//  用法
//    ./hc_sr04p                                  # 连续打印 左/中/右 三路距离
//    ./hc_sr04p --temp 27.5 --raw               # 指定气温，附加原始值
//    ./hc_sr04p --sensor left                   # 只跑左路（调试 / 单路标定）
//    ./hc_sr04p --stats 100 --sensor right      # 看右路抖动
//    ./hc_sr04p --calib 100.0 --sensor center   # 在 1 m 处采一组，追加到 calib.csv
//    ./hc_sr04p --fit calib.csv                  # 最小二乘拟合，给出 --k / --b
//    ./hc_sr04p --k 0.9982 --b -1.34            # 应用标定参数
//    ./hc_sr04p --rt --cpu 4                    # 提优先级并绑到 A76 大核
// ============================================================================

#include "hc_sr04p.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <sched.h>
#include <time.h>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

std::string errnoStr() { return std::string(std::strerror(errno)); }

void sleepMs(int ms) {
    if (ms <= 0) return;
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = static_cast<long>(ms % 1000) * 1000000L;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR && !g_stop) {}
}

// ---------------------------------------------------------------- 参数
struct CliOptions {
    int chip1 = 1;
    int chip4 = 4;
    int sensor = -1;        // -1 = 三路全开；0/1/2 = 左/中/右
    int gap_ms = 20;
    double temp_c = 25.0;
    int samples = 9;
    int interval_ms = 80;
    int timeout_ms = 40;
    int cpu = -1;
    bool rt = false;
    int stats_n = 0;
    double calib_true_cm = 0.0;
    bool do_calib = false;
    std::string calib_file = "calib.csv";
    std::string fit_file;
    double k = 1.0;
    double b = 0.0;
    bool show_raw = false;
    int count = 0;          // 0 = 无限循环
};

std::vector<int> activeSensors(int sensor) {
    if (sensor >= 0) return {sensor};
    return {hcsr04p::Left, hcsr04p::Center, hcsr04p::Right};
}

bool parseSensor(const std::string& s, int& out) {
    if (s == "left" || s == "l" || s == "左" || s == "0") {
        out = hcsr04p::Left;
    } else if (s == "center" || s == "middle" || s == "c" || s == "m" ||
               s == "中" || s == "1") {
        out = hcsr04p::Center;
    } else if (s == "right" || s == "r" || s == "右" || s == "2") {
        out = hcsr04p::Right;
    } else {
        return false;
    }
    return true;
}

void printHelp(const char* prog) {
    std::printf(
        "用法: %s [选项]\n\n"
        "三路接线（库里改引脚见 hc_sr04p.h 的 defaultSensors()）：\n"
        "  左  Trig GPIO1_A2 (chip1:2)    Echo GPIO1_A4 (chip1:4)\n"
        "  中  Trig GPIO1_A6 (chip1:6)    Echo GPIO1_D7 (chip1:31)\n"
        "  右  Trig GPIO4_A7 (chip4:7)    Echo GPIO4_A6 (chip4:6)\n\n"
        "  --chip1 N          GPIO1_* 所在 gpiochip 编号（默认 1）\n"
        "  --chip4 N          GPIO4_* 所在 gpiochip 编号（默认 4）\n"
        "  --sensor NAME      只跑一路：left|center|right / 左|中|右 / 0|1|2\n"
        "                     默认三路全跑（仅接了一路时用这个选项调试）\n"
        "  --gap MS           同一轮内相邻两路触发间隔 ms（默认 20，抑制串扰）\n"
        "  --temp C           环境气温，用于声速补偿（默认 25.0，务必填实测值）\n"
        "  --samples N        每轮每路采样次数，奇数（默认 9）\n"
        "  --interval MS      轮与轮之间的间隔 ms，必须 >= 60（默认 80）\n"
        "  --timeout MS       单次回波超时 ms（默认 40，约 6.8 m）\n"
        "  --rt               尝试 SCHED_FIFO 实时调度（需 root）\n"
        "  --cpu N            绑核到 CPU N（如 RK3588 的 4~7 是 A76 大核）\n"
        "  --stats N          抖动诊断：逐路采 N 次后打印统计，不进入循环\n"
        "  --calib TRUE_CM    在已知距离 TRUE_CM 处采一组，追加到标定文件\n"
        "                     必须配合 --sensor 指定被标定的那一路\n"
        "  --calib-file FILE  标定文件路径（默认 calib.csv）\n"
        "  --fit FILE         最小二乘拟合标定文件，输出建议的 --k / --b\n"
        "  --k K --b B        应用标定：真值 = K * 读数 + B（三路共用）\n"
        "  --raw              额外打印原始回波宽度（µs）与有效样本数\n"
        "  --count N          打印 N 轮后退出（默认一直跑）\n"
        "  --help             显示本帮助\n\n"
        "精度要点：温度补偿是最大误差源（10°C 温差在 1 m 处偏 15 mm），\n"
        "          必须挂 DS18B20 / SHT30 实测气温后传入 --temp。\n",
        prog);
}

bool needValue(int argc, char** argv, int& i, const char* key, double& out) {
    if (i + 1 >= argc) {
        std::fprintf(stderr, "%s 缺少参数值\n", key);
        return false;
    }
    out = std::atof(argv[++i]);
    return true;
}

bool parseArgs(int argc, char** argv, CliOptions& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        double d = 0.0;
        if (a == "--help" || a == "-h") {
            printHelp(argv[0]);
            std::exit(0);
        } else if (a == "--chip1") {
            if (!needValue(argc, argv, i, "--chip1", d)) return false;
            o.chip1 = static_cast<int>(d);
        } else if (a == "--chip4") {
            if (!needValue(argc, argv, i, "--chip4", d)) return false;
            o.chip4 = static_cast<int>(d);
        } else if (a == "--sensor") {
            if (i + 1 >= argc) { std::fprintf(stderr, "--sensor 缺少值\n"); return false; }
            if (!parseSensor(argv[++i], o.sensor)) {
                std::fprintf(stderr, "--sensor 只能是 left|center|right / 左|中|右 / 0|1|2\n");
                return false;
            }
        } else if (a == "--gap") {
            if (!needValue(argc, argv, i, "--gap", d)) return false;
            o.gap_ms = static_cast<int>(d);
        } else if (a == "--chip" || a == "--trig" || a == "--echo") {
            std::fprintf(stderr,
                         "%s 已移除：三路引脚见 hc_sr04p.h 的 defaultSensors()，\n"
                         "  用 --chip1 / --chip4 改 gpiochip 编号，用 --sensor 选单路。\n",
                         a.c_str());
            return false;
        } else if (a == "--temp") {
            if (!needValue(argc, argv, i, "--temp", d)) return false;
            o.temp_c = d;
        } else if (a == "--samples") {
            if (!needValue(argc, argv, i, "--samples", d)) return false;
            o.samples = static_cast<int>(d);
        } else if (a == "--interval") {
            if (!needValue(argc, argv, i, "--interval", d)) return false;
            o.interval_ms = static_cast<int>(d);
        } else if (a == "--timeout") {
            if (!needValue(argc, argv, i, "--timeout", d)) return false;
            o.timeout_ms = static_cast<int>(d);
        } else if (a == "--cpu") {
            if (!needValue(argc, argv, i, "--cpu", d)) return false;
            o.cpu = static_cast<int>(d);
        } else if (a == "--rt") {
            o.rt = true;
        } else if (a == "--stats") {
            if (!needValue(argc, argv, i, "--stats", d)) return false;
            o.stats_n = static_cast<int>(d);
        } else if (a == "--calib") {
            if (!needValue(argc, argv, i, "--calib", d)) return false;
            o.calib_true_cm = d;
            o.do_calib = true;
        } else if (a == "--calib-file") {
            if (i + 1 >= argc) { std::fprintf(stderr, "--calib-file 缺少值\n"); return false; }
            o.calib_file = argv[++i];
        } else if (a == "--fit") {
            if (i + 1 >= argc) { std::fprintf(stderr, "--fit 缺少值\n"); return false; }
            o.fit_file = argv[++i];
        } else if (a == "--k") {
            if (!needValue(argc, argv, i, "--k", d)) return false;
            o.k = d;
        } else if (a == "--b") {
            if (!needValue(argc, argv, i, "--b", d)) return false;
            o.b = d;
        } else if (a == "--raw") {
            o.show_raw = true;
        } else if (a == "--count") {
            if (!needValue(argc, argv, i, "--count", d)) return false;
            o.count = static_cast<int>(d);
        } else {
            std::fprintf(stderr, "未知参数: %s（用 --help 看用法）\n", a.c_str());
            return false;
        }
    }
    if (o.samples % 2 == 0) o.samples += 1;      // 中值需要奇数个样本
    if (o.samples < 3) o.samples = 3;            // 鲁棒统计至少要 3 个有效样本
    if (o.interval_ms < 60) o.interval_ms = 60;  // 模块规格要求 >= 60 ms
    if (o.gap_ms < 0) o.gap_ms = 0;
    if (o.chip1 < 0 || o.chip4 < 0) {
        std::fprintf(stderr, "--chip1 / --chip4 必须是非负整数\n");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------- 调度
void tuneScheduler(const CliOptions& o) {
    if (o.cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(o.cpu, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) {
            std::fprintf(stderr, "警告: 绑核到 CPU %d 失败: %s\n", o.cpu, errnoStr().c_str());
        }
    }
    if (o.rt) {
        struct sched_param sp;
        std::memset(&sp, 0, sizeof(sp));
        sp.sched_priority = 80;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            std::fprintf(stderr,
                         "警告: 设置 SCHED_FIFO 失败（多半是没 root）: %s\n"
                         "      本方案不依赖它，内核时间戳仍然有效。\n",
                         errnoStr().c_str());
        }
    }
}

// ---------------------------------------------------------------- 标定
int doFit(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "r");
    if (f == nullptr) {
        std::fprintf(stderr, "打不开 %s: %s\n", path.c_str(), errnoStr().c_str());
        return 1;
    }

    std::vector<double> x_meas;
    std::vector<double> y_true;
    char line[512];
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        double t = 0.0;
        double m = 0.0;
        if (std::sscanf(line, "%lf,%lf", &t, &m) == 2) {
            x_meas.push_back(m);
            y_true.push_back(t);
        }
    }
    std::fclose(f);

    const size_t n = x_meas.size();
    if (n < 2) {
        std::fprintf(stderr,
                     "%s 里只有 %zu 组有效数据，至少需要 2 组。\n"
                     "先用 ./hc_sr04p --calib 100.0 --sensor left 在若干已知距离各采一组。\n",
                     path.c_str(), n);
        return 1;
    }

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sx += x_meas[i];
        sy += y_true[i];
        sxx += x_meas[i] * x_meas[i];
        sxy += x_meas[i] * y_true[i];
    }
    const double dn = static_cast<double>(n);
    const double denom = dn * sxx - sx * sx;
    if (std::fabs(denom) < 1e-12) {
        std::fprintf(stderr, "数据点距离几乎相同，无法拟合，请在不同距离各采一组。\n");
        return 1;
    }
    const double k = (dn * sxy - sx * sy) / denom;
    const double b = (sy - k * sx) / dn;

    double ss_res = 0.0, ss_tot = 0.0;
    const double y_mean = sy / dn;
    double worst = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double pred = k * x_meas[i] + b;
        const double res = y_true[i] - pred;
        ss_res += res * res;
        ss_tot += (y_true[i] - y_mean) * (y_true[i] - y_mean);
        worst = std::max(worst, std::fabs(res));
    }
    const double r2 = (ss_tot > 1e-12) ? (1.0 - ss_res / ss_tot) : 1.0;

    std::printf("标定数据 %zu 组\n", n);
    std::printf("  真值 = k × 读数 + b\n");
    std::printf("  k = %.6f\n", k);
    std::printf("  b = %.4f cm\n", b);
    std::printf("  R² = %.6f\n", r2);
    std::printf("  标定后最大残差 = %.3f cm\n\n", worst);
    std::printf("用法: ./hc_sr04p --k %.6f --b %.4f\n", k, b);
    if (r2 < 0.99) {
        std::printf("\n提醒: R² 偏低，说明数据一致性差。常见原因：采样时目标面移动、\n"
                    "      量尺与探头参考面不统一、气温变化未记录。\n");
    }
    return 0;
}

int appendCalib(const std::string& path, double true_cm, double measured_cm) {
    bool need_header = true;
    if (std::FILE* probe = std::fopen(path.c_str(), "r")) {
        char buf[256];
        if (std::fgets(buf, sizeof(buf), probe) != nullptr && std::strlen(buf) > 3) {
            need_header = false;
        }
        std::fclose(probe);
    }
    std::FILE* f = std::fopen(path.c_str(), "a");
    if (f == nullptr) {
        std::fprintf(stderr, "写不了 %s: %s\n", path.c_str(), errnoStr().c_str());
        return 1;
    }
    if (need_header) std::fprintf(f, "true_cm,measured_cm\n");
    std::fprintf(f, "%.3f,%.3f\n", true_cm, measured_cm);
    std::fclose(f);
    return 0;
}

}  // namespace

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
    CliOptions opt;
    if (!parseArgs(argc, argv, opt)) return 2;

    if (!opt.fit_file.empty()) return doFit(opt.fit_file);

    if (opt.do_calib && opt.sensor < 0) {
        std::fprintf(stderr,
                     "标定必须指定单个传感器：--sensor left|center|right\n"
                     "（三路读数不同，混写进同一个标定文件没有意义）\n");
        return 2;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    tuneScheduler(opt);

    // 只把 CLI 参数映射到库配置，测距逻辑全在 hcsr04p::UltrasonicArray 里
    hcsr04p::Config cfg;
    cfg.sensors = hcsr04p::defaultSensors(opt.chip1, opt.chip4);
    cfg.samples = opt.samples;
    cfg.interval_ms = opt.interval_ms;
    cfg.gap_ms = opt.gap_ms;
    cfg.timeout_ms = opt.timeout_ms;
    cfg.temp_c = opt.temp_c;
    cfg.k = opt.k;
    cfg.b = opt.b;

    const std::vector<int> active = activeSensors(opt.sensor);
    hcsr04p::UltrasonicArray dev;
    std::string err;
    if (!dev.open(cfg, active, err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }

    std::printf("已就绪：");
    for (size_t j = 0; j < active.size(); ++j) {
        const hcsr04p::SensorConfig& c = dev.sensorConfig(active[j]);
        std::printf("%s[%s Trig %s=gpiochip%d:%u / Echo %s=gpiochip%d:%u]",
                    j ? "  " : "", c.name.c_str(), c.trig.name.c_str(), c.trig.chip,
                    c.trig.offset, c.echo.name.c_str(), c.echo.chip, c.echo.offset);
    }
    std::printf("\n每轮每路 %d 次采样，轮间隔 %d ms，路间隔 %d ms，气温 %.1f °C\n\n",
                opt.samples, opt.interval_ms, opt.gap_ms, opt.temp_c);

    const double v = hcsr04p::speedCmUs(opt.temp_c);

    // ---- 抖动诊断模式（逐路采原始宽度后统计）
    if (opt.stats_n > 0) {
        int ok = 0;
        for (int idx : active) {
            const hcsr04p::SensorConfig& c = dev.sensorConfig(idx);
            std::vector<uint64_t> raw;
            std::printf("[%s] 采集中（%d 次，间隔 %d ms）...\n",
                        c.name.c_str(), opt.stats_n, opt.interval_ms);
            for (int i = 0; i < opt.stats_n && !g_stop; ++i) {
                uint64_t w = 0;
                std::string pulse_err;
                if (dev.pulse(idx, w, pulse_err)) {
                    raw.push_back(w);
                } else if (!pulse_err.empty()) {
                    std::fprintf(stderr, "[%s] GPIO 错误: %s\n",
                                 c.name.c_str(), pulse_err.c_str());
                    break;
                }
                sleepMs(opt.interval_ms);
            }
            if (raw.empty()) {
                std::fprintf(stderr,
                             "[%s] 一次有效回波都没有。检查目标是否在量程内、是否垂直、\n"
                             "      供电是否 3.3V 且共地、5V 供电时 Echo 是否分压。\n",
                             c.name.c_str());
                continue;
            }
            bool outlier_removed = false;
            const hcsr04p::Stats s = hcsr04p::computeStats(raw, outlier_removed);
            std::printf("\n[%s] 有效样本 %d / %d", c.name.c_str(), s.n, opt.stats_n);
            if (outlier_removed) std::printf("（已剔除野值后统计）");
            std::printf("\n");
            std::printf("  回波宽度 中值 %.2f µs  均值 %.2f µs  标准差 %.3f µs\n",
                        s.median_us, s.mean_us, s.stdev_us);
            std::printf("  回波宽度 最小 %.2f  p1 %.2f  p99 %.2f  最大 %.2f µs\n",
                        s.min_us, s.p1_us, s.p99_us, s.max_us);
            std::printf("  折算距离 中值 %.3f cm  标准差 %.3f mm\n",
                        s.median_us * v / 2.0, s.stdev_us * v / 2.0 * 10.0);
            std::printf("  时间极差 %.2f µs → 距离极差 %.3f mm\n\n",
                        s.max_us - s.min_us, (s.max_us - s.min_us) * v / 2.0 * 10.0);
            ++ok;
        }
        std::printf("判读：标准差 < 2 mm 已达标；出现 >10 mm 的极值说明中断延迟偏大，\n"
                    "      或三路之间存在超声串扰（可加大 --gap）。\n");
        return ok > 0 ? 0 : 1;
    }

    // ---- 连续测量（含一次性标定采样）
    if (opt.do_calib) opt.count = 1;

    int printed = 0;
    while (!g_stop) {
        hcsr04p::Reading r;
        dev.measure(r, [] { return g_stop != 0; });

        std::string line;
        for (int idx : active) {
            const hcsr04p::Measurement& m = r[idx];
            const hcsr04p::SensorConfig& c = dev.sensorConfig(idx);
            char cell[128];
            if (m.ok()) {
                if (opt.show_raw) {
                    std::snprintf(cell, sizeof(cell), "%8.3f cm (%8.2f µs, %d/%d)",
                                  m.distance_cm, static_cast<double>(m.width_ns) * 1e-3,
                                  m.valid_samples, m.total_samples);
                } else {
                    std::snprintf(cell, sizeof(cell), "%8.3f cm", m.distance_cm);
                }
            } else if (m.status == hcsr04p::Status::Error) {
                std::snprintf(cell, sizeof(cell), "错误 (详情见 stderr)");
            } else {
                std::snprintf(cell, sizeof(cell), "%s",
                              opt.show_raw ? "--  (无回波, 有效 0)" : "--  (无回波)");
            }
            char seg[192];
            std::snprintf(seg, sizeof(seg), "[%s] %-28s  ", c.name.c_str(), cell);
            line += seg;
        }
        std::printf("%s\n", line.c_str());
        std::fflush(stdout);

        for (int idx : active) {
            if (r[idx].status == hcsr04p::Status::Error) {
                std::fprintf(stderr, "[%s] GPIO 错误，停止测量: %s\n",
                             dev.sensorConfig(idx).name.c_str(), r[idx].error.c_str());
                return 1;
            }
        }

        if (opt.do_calib) {
            const int idx = active[0];
            if (!r[idx].ok()) {
                std::printf("本轮无有效回波，未写入标定文件。\n");
            } else if (appendCalib(opt.calib_file, opt.calib_true_cm,
                                   r[idx].distance_cm) == 0) {
                std::printf("已追加到 %s: [%s] 真值 %.3f cm, 读数 %.3f cm\n"
                            "换个距离再采一组，然后用 --fit %s 拟合。\n",
                            opt.calib_file.c_str(),
                            dev.sensorConfig(idx).name.c_str(),
                            opt.calib_true_cm, r[idx].distance_cm,
                            opt.calib_file.c_str());
            }
        }

        ++printed;
        if (opt.count > 0 && printed >= opt.count) break;
        sleepMs(200);
    }
    return 0;
}
