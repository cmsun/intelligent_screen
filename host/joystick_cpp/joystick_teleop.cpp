// joystick_cpp：自定义串口链路的上位机手柄节点（feature/custom-serial 分支）。
// 功能与 master 分支 joystick_py/joystick_teleop.py 保持一致，区别在于：
//   - 通信从 ROS2/micro-ROS 话题改为自定义二进制串口帧（见 serial_protocol.hpp）
//   - 手柄从 pygame 改为 Linux joystick API（/dev/input/jsN）
//   - 全部日志/屏幕输出使用 spdlog
//
// 构建：cmake -S . -B build && cmake --build build -j
// 运行：JOYSTICK_DISPLAY=0 ./build/joystick_teleop --serial /dev/ttyS4 --joy 0

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "serial_protocol.hpp"

namespace
{
std::atomic<bool> g_running{true};

void on_signal(int)
{
    g_running.store(false);
}

struct Config
{
    std::string serial_path = "/dev/ttyS4";
    int joy_index = 0;
    double publish_hz = 20.0;
    double deadzone = 0.08;
    double linear_max = 0.6;
    double angular_max = 2.0;
    int linear_axis = 1;
    int angular_axis = 0;
    double linear_sign = -1.0;   // 摇杆上推为负 -> 前进为正
    double angular_sign = 1.0;
    int start_button = 11;
    int speed_up_button = 6;
    int speed_down_button = 8;
    int estop_button = -1;
    int servo_axis = 3;
    double servo_max_speed_deg_s = 60.0;
    int servo_move_time_ms = 300;
    int servo_move_time_step_ms = 50;
    int servo_move_time_up_button = 7;
    int servo_move_time_down_button = 9;
    double servo_sign = -1.0;
    double servo_min_deg = 0.0;
    double servo_max_deg = 170.0;
    bool servo_enabled = true;
    int offset_x_button = 3;     // X
    int offset_y_button = 4;     // Y
    int offset_up_button = 1;    // B：+0.1A
    int offset_down_button = 0;  // A：-0.1A
    double offset_step_a = 0.1;
    bool display = true;
};

double apply_deadzone(double value, double dz)
{
    if (std::fabs(value) <= dz)
    {
        return 0.0;
    }
    // 去掉死区后重新归一化，避免越过死区时速度突然跳变
    return (value - (value > 0.0 ? dz : -dz)) / (1.0 - dz);
}

const char *yn(bool value)
{
    return value ? "Y" : "N";
}

const char *floor_text(bool detected)
{
    // 与 ESP32 一致：true = detected（有地面），false = lost（悬空，触发防跌落保护）
    return detected ? "detected" : "lost";
}

const char *name_of(const char *const *table, size_t size, int index)
{
    return (index >= 0 && static_cast<size_t>(index) < size) ? table[index] : "unknown";
}

class SerialPort
{
public:
    ~SerialPort()
    {
        close();
    }

    bool open(const std::string &path, int baud)
    {
        _fd = ::open(path.c_str(), O_RDWR | O_NOCTTY);
        if (_fd < 0)
        {
            return false;
        }
        termios tio{};
        if (::tcgetattr(_fd, &tio) != 0)
        {
            close();
            return false;
        }
        ::cfmakeraw(&tio);
        const speed_t speed = (baud == 9600)    ? B9600
                              : (baud == 57600)  ? B57600
                              : (baud == 921600) ? B921600
                                                 : B115200;
        ::cfsetispeed(&tio, speed);
        ::cfsetospeed(&tio, speed);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tio.c_cflag |= CS8;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (::tcsetattr(_fd, TCSANOW, &tio) != 0)
        {
            close();
            return false;
        }
        ::tcflush(_fd, TCIOFLUSH);
        return true;
    }

    void close()
    {
        if (_fd >= 0)
        {
            ::close(_fd);
            _fd = -1;
        }
    }

    int fd() const { return _fd; }

    ssize_t read(uint8_t *buf, size_t n) const
    {
        return ::read(_fd, buf, n);
    }

    bool write_all(const uint8_t *buf, size_t n) const
    {
        size_t done = 0;
        while (done < n)
        {
            const ssize_t w = ::write(_fd, buf + done, n - done);
            if (w > 0)
            {
                done += static_cast<size_t>(w);
            }
            else if (w < 0 && errno == EINTR)
            {
                continue;
            }
            else
            {
                return false;
            }
        }
        return true;
    }

private:
    int _fd = -1;
};

class Joystick
{
public:
    ~Joystick()
    {
        close();
    }

    bool open(int index)
    {
        close();
        const std::string path = "/dev/input/js" + std::to_string(index);
        _fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (_fd < 0)
        {
            return false;
        }
        char name[128] = {};
        if (::ioctl(_fd, JSIOCGNAME(sizeof(name)), name) >= 0)
        {
            _name = name;
        }
        unsigned char axes = 0;
        unsigned char buttons = 0;
        ::ioctl(_fd, JSIOCGAXES, &axes);
        ::ioctl(_fd, JSIOCGBUTTONS, &buttons);
        _axes.assign(axes, 0.0);
        _buttons.assign(buttons, 0);

        // Linux joystick 把十字键映射成 ABS_HAT0X/Y 轴，按 axis map 找出索引，
        // 对齐 Python 版 pygame.get_hat(0) 的语义
        uint8_t axmap[ABS_MAX + 1] = {};
        if (::ioctl(_fd, JSIOCGAXMAP, axmap) >= 0)
        {
            for (int i = 0; i < static_cast<int>(axes); ++i)
            {
                if (axmap[i] == ABS_HAT0X) _hat_x = i;
                else if (axmap[i] == ABS_HAT0Y) _hat_y = i;
            }
        }
        return true;
    }

    void close()
    {
        if (_fd >= 0)
        {
            ::close(_fd);
            _fd = -1;
        }
    }

    bool ok() const { return _fd >= 0; }
    const std::string &name() const { return _name; }
    size_t axes() const { return _axes.size(); }
    size_t buttons() const { return _buttons.size(); }

    void poll()
    {
        if (_fd < 0)
        {
            return;
        }
        js_event ev{};
        while (::read(_fd, &ev, sizeof(ev)) == static_cast<ssize_t>(sizeof(ev)))
        {
            if ((ev.type & JS_EVENT_AXIS) && ev.number < _axes.size())
            {
                _axes[ev.number] = static_cast<double>(ev.value) / 32767.0;
            }
            else if ((ev.type & JS_EVENT_BUTTON) && ev.number < _buttons.size())
            {
                _buttons[ev.number] = ev.value ? 1 : 0;
            }
        }
    }

    double axis(int index) const
    {
        return (index >= 0 && static_cast<size_t>(index) < _axes.size()) ? _axes[index] : 0.0;
    }

    bool button(int index) const
    {
        return index >= 0 && static_cast<size_t>(index) < _buttons.size() && _buttons[index] != 0;
    }

    std::pair<int, int> hat() const
    {
        auto discrete = [](double v) { return v > 0.3 ? 1 : (v < -0.3 ? -1 : 0); };
        return {(_hat_x >= 0) ? discrete(axis(_hat_x)) : 0,
                (_hat_y >= 0) ? discrete(axis(_hat_y)) : 0};
    }

private:
    int _fd = -1;
    int _hat_x = -1;
    int _hat_y = -1;
    std::string _name;
    std::vector<double> _axes;
    std::vector<uint8_t> _buttons;
};

struct Snapshots
{
    std::mutex mutex;
    bool imu_valid = false;
    chassis_proto::ImuStatus imu{};
    bool xl_valid = false;
    chassis_proto::Xl9535Status xl{};
    bool ir_valid = false;
    chassis_proto::IrStatus ir{};
    bool motion_valid = false;
    chassis_proto::MotionStatus motion{};
    bool battery_valid = false;
    chassis_proto::BatteryStatus battery{};
    bool servo_valid = false;
    chassis_proto::ServoStatus servo{};
    bool servo_ready_latched = false;
    double servo_target_deg = 0.0;
    std::chrono::steady_clock::time_point last_frame{};
    bool link_ok = false;
};

class Teleop
{
public:
    Teleop(Config cfg, std::shared_ptr<spdlog::logger> screen)
        : _cfg(std::move(cfg)), _screen(std::move(screen))
    {
        _servo_move_time_ms = _cfg.servo_move_time_ms;
    }

    bool start()
    {
        if (!_serial.open(_cfg.serial_path, static_cast<int>(chassis_proto::kBaudRate)))
        {
            spdlog::error("open serial {} failed: {}", _cfg.serial_path, std::strerror(errno));
            return false;
        }
        if (!_joy.open(_cfg.joy_index))
        {
            spdlog::warn("no joystick found, velcmd muted");
        }
        else
        {
            spdlog::info("joystick[{}] {} axes={} buttons={}", _cfg.joy_index, _joy.name(),
                         _joy.axes(), _joy.buttons());
        }
        spdlog::info("custom serial link {} @ {} baud; press START(button {}) to enable",
                     _cfg.serial_path, chassis_proto::kBaudRate, _cfg.start_button);
        spdlog::info("servo open-loop: axis{}, {:.1f} deg/s, {:.1f}~{:.1f} deg, move_time_ms={}",
                     _cfg.servo_axis, _cfg.servo_max_speed_deg_s, _cfg.servo_min_deg,
                     _cfg.servo_max_deg, _cfg.servo_move_time_ms);
        _rx_thread = std::thread(&Teleop::rx_loop, this);
        return true;
    }

    void run()
    {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(1.0 / std::max(1.0, _cfg.publish_hz)));
        auto next_tick = clock::now() + period;
        auto last_servo_tick = clock::now();
        auto last_ping = clock::now();

        while (g_running.load())
        {
            _joy.poll();
            handle_buttons();
            handle_offset_buttons();
            update_offset_calibration();

            const auto now = clock::now();
            const double dt = std::min(std::max(std::chrono::duration<double>(now - last_servo_tick).count(), 0.0), 0.1);
            last_servo_tick = now;

            double linear = 0.0;
            double angular = 0.0;
            const bool active = _enabled && !_estop && _joy.ok();
            if (active)
            {
                // 沿用 Python 版逻辑：先十字键，后摇杆；摇杆有输入时覆盖十字键
                const auto [hat_x, hat_y] = _joy.hat();
                if (hat_x != 0) angular = hat_x * _angular_scale;
                if (hat_y != 0) linear = hat_y * _linear_scale;

                const double a_lin = apply_deadzone(_joy.axis(_cfg.linear_axis), _cfg.deadzone);
                const double a_ang = apply_deadzone(_joy.axis(_cfg.angular_axis), _cfg.deadzone);
                if (a_lin != 0.0) linear = _cfg.linear_sign * a_lin * _linear_scale;
                if (a_ang != 0.0) angular = _cfg.angular_sign * a_ang * _angular_scale;
                linear = std::clamp(linear, -_cfg.linear_max, _cfg.linear_max);
                angular = std::clamp(angular, -_cfg.angular_max, _cfg.angular_max);
            }

            const bool cmd_is_zero = (linear == 0.0 && angular == 0.0);
            bool chassis_stopped = false;
            {
                std::lock_guard<std::mutex> lock(_snap.mutex);
                if (_snap.motion_valid)
                {
                    chassis_stopped = std::fabs(_snap.motion.linear_speed_cur) <= 1e-3 &&
                                      std::fabs(_snap.motion.angular_speed_cur) <= 1e-3;
                }
            }
            // 零速且底盘已停时不再重复下发，避免挤占串口带宽
            const bool sent = active && !(cmd_is_zero && chassis_stopped);
            if (sent)
            {
                chassis_proto::VelCmd cmd{static_cast<float>(linear), static_cast<float>(angular)};
                send(chassis_proto::kCmdVel, cmd);
            }

            if (active)
            {
                update_servo(dt);
            }

            // 手柄静止时不发 velcmd，靠 PING 维持 ESP32 侧链路保活
            if (now - last_ping >= std::chrono::milliseconds(500))
            {
                last_ping = now;
                send_raw(chassis_proto::kCmdPing, nullptr, 0);
            }

            if (_cfg.display)
            {
                render(linear, angular, sent);
            }

            std::this_thread::sleep_until(next_tick);
            next_tick += period;
            if (next_tick < clock::now())
            {
                next_tick = clock::now() + period;
            }
        }
    }

    void stop()
    {
        g_running.store(false);
        if (_rx_thread.joinable())
        {
            _rx_thread.join();
        }
        // 退出前下发零速，避免底盘继续跑最后一条指令（ESP32 侧看门狗也会兜底）
        chassis_proto::VelCmd cmd{0.0f, 0.0f};
        send(chassis_proto::kCmdVel, cmd);
    }

private:
    template <typename T>
    void send(uint8_t msg_id, const T &msg)
    {
        send_raw(msg_id, &msg, static_cast<uint8_t>(sizeof(T)));
    }

    void send_raw(uint8_t msg_id, const void *payload, uint8_t len)
    {
        uint8_t buf[chassis_proto::kMaxPayload + 5] = {};
        const size_t n = chassis_proto::build_frame(buf, msg_id, payload, len);
        if (!_serial.write_all(buf, n))
        {
            spdlog::warn("serial write failed: {}", std::strerror(errno));
        }
    }

    void rx_loop()
    {
        chassis_proto::FrameParser parser;
        uint8_t payload[chassis_proto::kMaxPayload] = {};
        uint8_t payload_len = 0;
        uint8_t raw[256] = {};
        pollfd pfd{_serial.fd(), POLLIN, 0};
        while (g_running.load())
        {
            const int rc = ::poll(&pfd, 1, 20);
            if (rc <= 0)
            {
                continue;
            }
            const ssize_t n = _serial.read(raw, sizeof(raw));
            for (ssize_t i = 0; i < n; ++i)
            {
                const uint8_t id = parser.feed(raw[i], payload, sizeof(payload), payload_len);
                if (id != 0)
                {
                    handle_frame(id, payload, payload_len);
                }
            }
        }
    }

    void handle_frame(uint8_t id, const uint8_t *payload, uint8_t len)
    {
        std::lock_guard<std::mutex> lock(_snap.mutex);
        _snap.last_frame = std::chrono::steady_clock::now();
        _snap.link_ok = true;
        switch (id)
        {
        case chassis_proto::kStatusImu:
            if (len == sizeof(chassis_proto::ImuStatus))
            {
                std::memcpy(&_snap.imu, payload, len);
                _snap.imu_valid = true;
            }
            break;
        case chassis_proto::kStatusXl9535:
            if (len == sizeof(chassis_proto::Xl9535Status))
            {
                std::memcpy(&_snap.xl, payload, len);
                _snap.xl_valid = true;
            }
            break;
        case chassis_proto::kStatusIr:
            if (len == sizeof(chassis_proto::IrStatus))
            {
                std::memcpy(&_snap.ir, payload, len);
                _snap.ir_valid = true;
            }
            break;
        case chassis_proto::kStatusMotion:
            if (len == sizeof(chassis_proto::MotionStatus))
            {
                std::memcpy(&_snap.motion, payload, len);
                _snap.motion_valid = true;
            }
            break;
        case chassis_proto::kStatusBattery:
            if (len == sizeof(chassis_proto::BatteryStatus))
            {
                std::memcpy(&_snap.battery, payload, len);
                _snap.battery_valid = true;
            }
            break;
        case chassis_proto::kStatusServo:
            if (len == sizeof(chassis_proto::ServoStatus))
            {
                std::memcpy(&_snap.servo, payload, len);
                _snap.servo_valid = true;
                // 开环：位置反馈只在首次握手时锁存目标初值
                if (_snap.servo.ready && !_snap.servo_ready_latched)
                {
                    _snap.servo_ready_latched = true;
                    _snap.servo_target_deg = std::clamp(static_cast<double>(_snap.servo.angle_deg),
                                                        _cfg.servo_min_deg, _cfg.servo_max_deg);
                    spdlog::info("servo READY: id={} position={} angle={:.1f} deg voltage={:.2f} V; "
                                 "open-loop target locked at {:.1f} deg",
                                 _snap.servo.id, _snap.servo.position, _snap.servo.angle_deg,
                                 _snap.servo.voltage_v, _snap.servo_target_deg);
                }
            }
            break;
        case chassis_proto::kStatusPong:
            break;  // 链路保活应答，无需处理
        default:
            break;
        }
    }

    void handle_buttons()
    {
        if (!_joy.ok())
        {
            _enabled = false;
            return;
        }

        const bool start = _joy.button(_cfg.start_button);
        if (start && !_prev_start)
        {
            _enabled = !_enabled;
            spdlog::info("chassis {}", _enabled ? "ENABLED" : "DISABLED");
        }
        _prev_start = start;

        const bool up = _joy.button(_cfg.speed_up_button);
        if (up && !_prev_up)
        {
            _linear_scale = std::min(_cfg.linear_max, _linear_scale + 0.05);
            _angular_scale = std::min(_cfg.angular_max, _angular_scale + 0.10);
            spdlog::info("speed up: {:.2f} m/s, {:.2f} rad/s", _linear_scale, _angular_scale);
        }
        _prev_up = up;

        const bool down = _joy.button(_cfg.speed_down_button);
        if (down && !_prev_down)
        {
            _linear_scale = std::max(0.05, _linear_scale - 0.05);
            _angular_scale = std::max(0.10, _angular_scale - 0.10);
            spdlog::info("speed down: {:.2f} m/s, {:.2f} rad/s", _linear_scale, _angular_scale);
        }
        _prev_down = down;

        const bool mt_up = _joy.button(_cfg.servo_move_time_up_button);
        if (mt_up && !_prev_mt_up)
        {
            _servo_move_time_ms = std::min(65535, _servo_move_time_ms + _cfg.servo_move_time_step_ms);
            spdlog::info("servo move_time_ms up: {}", _servo_move_time_ms);
        }
        _prev_mt_up = mt_up;

        const bool mt_down = _joy.button(_cfg.servo_move_time_down_button);
        if (mt_down && !_prev_mt_down)
        {
            _servo_move_time_ms = std::max(0, _servo_move_time_ms - _cfg.servo_move_time_step_ms);
            spdlog::info("servo move_time_ms down: {}", _servo_move_time_ms);
        }
        _prev_mt_down = mt_down;

        // 急停是一次性的：触发后需重启节点，避免误碰恢复
        if (_joy.button(_cfg.estop_button) && !_estop)
        {
            _estop = true;
            _enabled = false;
            spdlog::error("ESTOP: velcmd muted, restart node to recover");
        }
    }

    void publish_offset(double value)
    {
        chassis_proto::BatteryOffsetCmd cmd{static_cast<float>(value)};
        send(chassis_proto::kCmdBatteryOffset, cmd);
        spdlog::info("send battery current offset {:.3f} A to ESP32", value);
    }

    void handle_offset_buttons()
    {
        // X+Y 组合键上升沿：进入标定模式；再次按下则把当前值下发给 ESP32 保存
        const bool combo = _joy.button(_cfg.offset_x_button) && _joy.button(_cfg.offset_y_button);
        if (combo && !_prev_offset_combo)
        {
            if (!_offset_mode)
            {
                _offset_mode = true;
                _offset_phase = "zeroing";
                _offset_target = 0.0;
                _offset_zero_sent_at = std::chrono::steady_clock::time_point{};
                _offset_hint = "已进入标定：先检查并清零 ESP32 零漂";
                spdlog::info("battery offset calibration: ENTER (X+Y), zeroing ESP32 offset");
            }
            else
            {
                publish_offset(_offset_target);
                _offset_mode = false;
                _offset_phase = "idle";
                _offset_hint = "已下发 " + std::to_string(_offset_target) + " A，等待 BatteryStatus 回读确认（NVS）";
                spdlog::info("battery offset calibration: SAVE {:.3f} A (X+Y)", _offset_target);
            }
        }
        _prev_offset_combo = combo;

        // 只有确认 ESP32 零漂已为 0 后，才允许 A/B 微调
        const bool up = _joy.button(_cfg.offset_up_button);
        const bool down = _joy.button(_cfg.offset_down_button);
        if (_offset_mode && _offset_phase == "ready")
        {
            if (up && !_prev_offset_up)
            {
                _offset_target = std::round((_offset_target + _cfg.offset_step_a) * 1000.0) / 1000.0;
                spdlog::info("battery offset target +{:.3f} -> {:.3f} A", _cfg.offset_step_a, _offset_target);
            }
            if (down && !_prev_offset_down)
            {
                _offset_target = std::round((_offset_target - _cfg.offset_step_a) * 1000.0) / 1000.0;
                spdlog::info("battery offset target -{:.3f} -> {:.3f} A", _cfg.offset_step_a, _offset_target);
            }
        }
        _prev_offset_up = up;
        _prev_offset_down = down;
    }

    void update_offset_calibration()
    {
        if (!_offset_mode || _offset_phase != "zeroing")
        {
            return;
        }
        bool valid = false;
        float offset = 0.0f;
        {
            std::lock_guard<std::mutex> lock(_snap.mutex);
            valid = _snap.battery_valid;
            offset = _snap.battery.current_offset;
        }
        if (!valid)
        {
            _offset_hint = "等待 BatteryStatus 上报";
            return;
        }
        if (std::fabs(offset) > 1e-6f)
        {
            // 每 3 秒重发一次 0，直到 ESP32 上报确认，避免每帧都写 NVS
            const auto now = std::chrono::steady_clock::now();
            if (_offset_zero_sent_at.time_since_epoch().count() == 0 ||
                now - _offset_zero_sent_at >= std::chrono::seconds(3))
            {
                _offset_zero_sent_at = now;
                publish_offset(0.0);
            }
            _offset_hint = "ESP32 当前零漂 " + std::to_string(offset) + " A，正在自动清零...";
            return;
        }
        _offset_phase = "ready";
        _offset_target = 0.0;
        _offset_hint = "零漂已为 0：记万用表读数 A1、屏幕 current 为 A2（A2 带符号）；"
                       "零漂值 = (|A1| × -1) - A2；B(+)/A(-) 调 target，再按 X+Y 保存";
        spdlog::info("battery offset calibration: ESP32 offset = 0, "
                     "offset = -abs(A1) - A2, use B(+)/A(-) then X+Y to save");
    }

    void update_servo(double dt)
    {
        double target = 0.0;
        {
            std::lock_guard<std::mutex> lock(_snap.mutex);
            if (!_cfg.servo_enabled || !_snap.servo_ready_latched)
            {
                return;
            }
            const double value = apply_deadzone(_joy.axis(_cfg.servo_axis), _cfg.deadzone);
            if (value == 0.0)
            {
                return;  // 推杆居中：目标保持不变，舵机停在原处
            }
            _snap.servo_target_deg = std::clamp(
                _snap.servo_target_deg + _cfg.servo_sign * value * _cfg.servo_max_speed_deg_s * dt,
                _cfg.servo_min_deg, _cfg.servo_max_deg);
            target = _snap.servo_target_deg;
        }
        chassis_proto::ServoCmd cmd{static_cast<float>(target),
                                    static_cast<uint16_t>(_servo_move_time_ms)};
        send(chassis_proto::kCmdServo, cmd);
    }

    void render(double linear, double angular, bool sent)
    {
        static const char *const kMotorStates[] = {"disable", "enable"};
        static const char *const kMotorModes[] = {"open", "current", "speed", "position"};
        static const char *const kBatteryStates[] = {"idle", "discharging", "charging",
                                                     "full", "low_battery", "over_current"};
        const auto has = [](uint8_t bits, uint8_t bit) { return (bits & bit) != 0; };

        std::string imu_text = "no data";
        std::string xl_text = "no data";
        std::string ir_text = "no data";
        std::string motion_text = "no data";
        std::string battery_text = "no data";
        bool servo_ready = false;
        double servo_target = 0.0;
        bool link_ok = false;
        {
            std::lock_guard<std::mutex> lock(_snap.mutex);
            link_ok = _snap.link_ok &&
                      (std::chrono::steady_clock::now() - _snap.last_frame < std::chrono::seconds(2));
            servo_ready = _snap.servo_ready_latched;
            servo_target = _snap.servo_target_deg;

            if (_snap.imu_valid)
            {
                std::ostringstream s;
                s << "roll=" << _snap.imu.roll_deg << " deg, pitch=" << _snap.imu.pitch_deg
                  << " deg, yaw=" << _snap.imu.yaw_deg << " deg, yaw_rate="
                  << _snap.imu.yaw_rate_deg_s << " deg/s";
                imu_text = s.str();
            }
            if (_snap.xl_valid)
            {
                std::ostringstream s;
                s << "floor L/LM/RM/R="
                  << floor_text(has(_snap.xl.flags, chassis_proto::kXlFloorLeft)) << "/"
                  << floor_text(has(_snap.xl.flags, chassis_proto::kXlFloorLeftMiddle)) << "/"
                  << floor_text(has(_snap.xl.flags, chassis_proto::kXlFloorRightMiddle)) << "/"
                  << floor_text(has(_snap.xl.flags, chassis_proto::kXlFloorRight))
                  << ", obstacle head up/down="
                  << yn(has(_snap.xl.flags, chassis_proto::kXlHeadUpObstacle)) << "/"
                  << yn(has(_snap.xl.flags, chassis_proto::kXlHeadDownObstacle))
                  << ", fan=" << yn(has(_snap.xl.flags, chassis_proto::kXlFanOn))
                  << ", int=" << yn(has(_snap.xl.flags, chassis_proto::kXlIntActive));
                xl_text = s.str();
            }
            if (_snap.ir_valid)
            {
                std::ostringstream s;
                if (_snap.ir.timestamp_ms == 0)
                {
                    s << "no frame yet, active L/R="
                      << yn(has(_snap.ir.flags, chassis_proto::kIrLeftActive)) << "/"
                      << yn(has(_snap.ir.flags, chassis_proto::kIrRightActive));
                }
                else
                {
                    s << "side=" << (_snap.ir.side == 0 ? "L" : "R")
                      << " addr=0x" << std::hex << static_cast<int>(_snap.ir.address)
                      << " cmd=0x" << static_cast<int>(_snap.ir.command)
                      << " raw=0x" << _snap.ir.raw_code << std::dec
                      << " valid=" << yn(has(_snap.ir.flags, chassis_proto::kIrAddressValid))
                      << " repeat=" << yn(has(_snap.ir.flags, chassis_proto::kIrRepeat))
                      << " t=" << _snap.ir.timestamp_ms << " ms, active L/R="
                      << yn(has(_snap.ir.flags, chassis_proto::kIrLeftActive)) << "/"
                      << yn(has(_snap.ir.flags, chassis_proto::kIrRightActive));
                }
                ir_text = s.str();
            }
            if (_snap.motion_valid)
            {
                std::ostringstream s;
                s << "cur=(" << _snap.motion.linear_speed_cur << " m/s, "
                  << _snap.motion.angular_speed_cur << " rad/s), rpm L/R="
                  << _snap.motion.left_motor_rpm << "/" << _snap.motion.right_motor_rpm
                  << ", state L/R="
                  << name_of(kMotorStates, 2, _snap.motion.left_motor_state) << "/"
                  << name_of(kMotorStates, 2, _snap.motion.right_motor_state)
                  << ", mode L/R="
                  << name_of(kMotorModes, 4, _snap.motion.left_motor_mode) << "/"
                  << name_of(kMotorModes, 4, _snap.motion.right_motor_mode)
                  << ", com_err L/R="
                  << yn(has(_snap.motion.comm_errors, chassis_proto::kMotionLeftCommError)) << "/"
                  << yn(has(_snap.motion.comm_errors, chassis_proto::kMotionRightCommError));
                motion_text = s.str();
            }
            if (_snap.battery_valid)
            {
                std::ostringstream s;
                s << "battery=" << _snap.battery.battery_voltage << " V, charger="
                  << _snap.battery.charger_voltage << " V, current=" << _snap.battery.current
                  << " A, power=" << _snap.battery.power << " W, soc=" << _snap.battery.soc
                  << " %, state=" << name_of(kBatteryStates, 6, _snap.battery.state)
                  << ", charge_en=" << yn(has(_snap.battery.flags, chassis_proto::kBatteryChargeEnabled))
                  << ", offset=" << _snap.battery.current_offset << " A";
                battery_text = s.str();
            }
        }

        std::ostringstream out;
        out << "\033[H\033[J";
        out << "=== [joystick / velcmd] (custom serial) ===\n";
        out << "  enabled: " << (_enabled ? "True" : "False")
            << ", estop: " << (_estop ? "True" : "False")
            << ", joystick: " << (_joy.ok() ? "ok" : "none")
            << ", link: " << (link_ok ? "ok" : "waiting") << "\n";
        out << "  scale: linear=" << _linear_scale << " m/s, angular=" << _angular_scale << " rad/s\n";
        out << "  cmd: linear=" << linear << " m/s, angular=" << angular
            << " rad/s, sent=" << yn(sent) << "\n";
        out << "=== [servo] ===\n";
        out << "  target: " << (servo_ready ? std::to_string(servo_target) + " deg" : "waiting") << "\n";
        out << "  move_time_ms: " << _servo_move_time_ms
            << " (step=" << _cfg.servo_move_time_step_ms << ")\n";
        out << "=== [IMU: euler + yaw rate] ===\n  " << imu_text << "\n";
        out << "=== [XL9535: gpio expander] ===\n  " << xl_text << "\n";
        out << "=== [IR: nec frame] ===\n  " << ir_text << "\n";
        out << "=== [Motion: chassis] ===\n  " << motion_text << "\n";
        out << "=== [Battery: bms] ===\n  " << battery_text << "\n";
        out << "=== [Battery offset calibration] ===\n";
        out << "  mode=" << (_offset_mode ? "ON [" + _offset_phase + "]" : "off")
            << ", target=" << _offset_target << " A | "
            << (_offset_hint.empty() ? "X+Y 进入零漂标定" : _offset_hint) << "\n";
        out << "  keys: X+Y(" << _cfg.offset_x_button << "+" << _cfg.offset_y_button
            << ") enter/save, B(" << _cfg.offset_up_button << ") +" << _cfg.offset_step_a
            << " A, A(" << _cfg.offset_down_button << ") -" << _cfg.offset_step_a << " A\n";
        out << "=== [buttons] START=" << _cfg.start_button << " enable/disable, "
            << _cfg.speed_up_button << "/" << _cfg.speed_down_button << " velcmd scale +/-, "
            << _cfg.servo_move_time_up_button << "/" << _cfg.servo_move_time_down_button
            << " move_time_ms +/-, estop=" << _cfg.estop_button << "\n";
        _screen->info("{}", out.str());
    }

    Config _cfg;
    std::shared_ptr<spdlog::logger> _screen;
    SerialPort _serial;
    Joystick _joy;
    Snapshots _snap;
    std::thread _rx_thread;

    bool _enabled = false;
    bool _estop = false;
    bool _prev_start = false;
    bool _prev_up = false;
    bool _prev_down = false;
    bool _prev_mt_up = false;
    bool _prev_mt_down = false;

    double _linear_scale = 0.20;
    double _angular_scale = 0.47;
    int _servo_move_time_ms = 300;

    bool _offset_mode = false;
    std::string _offset_phase = "idle";
    double _offset_target = 0.0;
    std::chrono::steady_clock::time_point _offset_zero_sent_at{};
    std::string _offset_hint;
    bool _prev_offset_combo = false;
    bool _prev_offset_up = false;
    bool _prev_offset_down = false;
};
} // namespace

int main(int argc, char **argv)
{
    auto console = spdlog::stdout_color_mt("teleop");
    spdlog::set_default_logger(console);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    spdlog::set_level(spdlog::level::info);

    Config cfg;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--serial" && i + 1 < argc)
        {
            cfg.serial_path = argv[++i];
        }
        else if (arg == "--joy" && i + 1 < argc)
        {
            cfg.joy_index = std::atoi(argv[++i]);
        }
        else if (arg == "--no-display")
        {
            cfg.display = false;
        }
        else if (arg == "--help" || arg == "-h")
        {
            spdlog::info("usage: joystick_teleop [--serial /dev/ttyS4] [--joy N] [--no-display]");
            return 0;
        }
        else
        {
            spdlog::warn("unknown argument: {}", arg);
        }
    }
    const char *env_display = std::getenv("JOYSTICK_DISPLAY");
    if (env_display != nullptr && std::strcmp(env_display, "0") == 0)
    {
        cfg.display = false;
    }

    // 屏幕状态显示单独用一个 logger，去掉时间戳/级别前缀，保证清屏输出不串行
    auto screen = spdlog::stdout_color_mt("screen");
    screen->set_pattern("%v");
    screen->set_level(spdlog::level::info);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    Teleop teleop(cfg, screen);
    if (!teleop.start())
    {
        return 1;
    }
    teleop.run();
    teleop.stop();
    spdlog::info("joystick_cpp stopped");
    spdlog::shutdown();
    return 0;
}
