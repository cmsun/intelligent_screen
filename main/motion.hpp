#ifndef __MOTION_H__
#define __MOTION_H__

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <ostream>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
// fmtlib自定义类型的字符串格式化需要包含<fmt/ostream.h>头文件
#include <fmt/ostream.h>

class MotionClass
{
public:
    MotionClass(void) = default;
    ~MotionClass(void) = default;
    void begin(void);
    void power_on(void);
    void set_chassis_speed(float linear, float angular);
    //自定义类型实现了std::ostream的operator<<函数后，fmtlib就可以支持自定义类型的字符串格式化，格式化自定义类型时要引入<fmt/ostream.h>头文件
    friend std::ostream &operator<<(std::ostream &os, const MotionClass &m);

    enum class MotorBusID
    {
        left = 1,  // 左电机
        right = 2, // 右电机
    };

    enum class ModeCmd
    {
        SetOpenMode = 0x00,          // 设定为开环模式，成功返回0x00
        SetCurrentMode = 0x01,       // 设定为电流模式，成功返回0x01
        SetSpeedMode = 0x02,         // 设定为速度模式，成功返回0x02
        SetPositionMode = 0x03,      // 设定为位置模式，成功返回0x03
        SetEnable = 0x08,            // 使能电机，成功返回0x01
        SetDisable = 0x09,           // 失能电机，成功返回0x01
        EnableDisConnectStop = 0x10, // enable通信断开3秒电机制动，成功返回0x02
        DisableDisConnectStop = 0x11, // disable通信断开3秒电机制动，成功返回0x02
        SetPID = 0xBD,               // 设定PID参数
    };

    enum class State
    {
        disable = 0, // 电机失能状态
        enable = 1,  // 电机使能状态
    };

    enum class DriveMode
    {
        open = 0x00,     // 开环驱动模式
        current = 0x01,  // 电流驱动模式
        speed = 0x02,    // 速度驱动模式
        position = 0x03, // 位置驱动模式
    };

private:
    bool _is_ready = false;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>> _left_motor_last_tx_time;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>> _right_motor_last_tx_time;
    State _left_motor_state = State::disable;
    State _right_motor_state = State::disable;
    DriveMode _left_motor_mode = MotionClass::DriveMode::open;
    DriveMode _right_motor_mode = MotionClass::DriveMode::open;
    bool _left_motor_disconnect_stop = false;
    bool _right_motor_disconnect_stop = false;
    bool _left_motor_pid_set = false;
    bool _right_motor_pid_set = false;
    uint8_t _left_motor_communication_error_count = 0;
    uint8_t _right_motor_communication_error_count = 0;
    bool _left_motor_communication_error = false;
    bool _right_motor_communication_error = false;
    float _linear_speed_acc = 4 * 3600;
    float _angular_speed_acc = 450 * 20 * 3.1415926f;
    float _linear_speed_set = 0.0f;
    float _angular_speed_set = 0.0f;
    float _linear_speed_cur = 0.0f;
    float _angular_speed_cur = 0.0f;
    int16_t _left_motor_rpm = 0;
    int16_t _right_motor_rpm = 0;
    uint32_t _last_plan_time = 0; // 上次规划时间，单位：毫秒
    std::vector<uint8_t> _rx_buffer;
    std::mutex _mutex;
    std::condition_variable _condition;
    QueueHandle_t _uart_queue = nullptr; // 电机 UART 事件队列
    static void uart_rx_task(void *arg);
    void mode_cmd_rx_data_process(MotorBusID id, ModeCmd cmd);
    void speed_set_rx_data_process(MotorBusID id);
    static float velocity_planning(float vel_cur, float vel_set, float acc, uint32_t interval);
    static std::pair<int16_t, int16_t> speed_convert_rpm(float linear, float angular);
    static void motion_task(void *arg);
};

//自定义类型实现了std::ostream的operator<<函数后，fmtlib就可以支持自定义类型的字符串格式化，格式化自定义类型时要引入<fmt/ostream.h>头文件
constexpr std::ostream &operator<<(std::ostream &os, const MotionClass::State &state)
{
    switch (state)
    {
    case MotionClass::State::disable:
        return os << "disable";
    case MotionClass::State::enable:
        return os << "enable";
    default:
        return os << "unknown state";
    }
}

constexpr std::ostream &operator<<(std::ostream &os, const MotionClass::DriveMode &mode)
{
    switch (mode)
    {
    case MotionClass::DriveMode::open:
        return os << "open";
    case MotionClass::DriveMode::current:
        return os << "current";
    case MotionClass::DriveMode::speed:
        return os << "speed";
    case MotionClass::DriveMode::position:
        return os << "position";
    default:
        return os << "unknown mode";
    }
}

extern MotionClass Motion;

#endif // __MOTION_H__
