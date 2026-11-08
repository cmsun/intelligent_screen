#include "motion.hpp"

#include "esplog.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

MotionClass Motion;

namespace
{
// ---- 电机控制板 UART / 电源 ----
constexpr uart_port_t kMotorUart = UART_NUM_1;
constexpr int kMotorUartTxPin = 18; // 电机控制板 UART TX（原 Arduino Serial1 TX）
constexpr int kMotorUartRxPin = 17; // 电机控制板 UART RX（原 Arduino Serial1 RX）
constexpr uint32_t kMotorUartBaud = 38400;
constexpr int kMotorPowerGpio = 39; // 电机电源使能引脚（原 pinMode(39, OUTPUT)）

constexpr uint8_t kCrcPolyReflected = 0x8C; // 0x31 按位反写，右下移实现用

// ---- 电机协议帧 ----
constexpr std::size_t kFrameSize = 10;
constexpr uint8_t kCmdMode = 0xA0;      // 模式 / 使能类指令
constexpr uint8_t kCmdSetPid = 0xBD;    // 设定 PID 参数
constexpr uint8_t kCmdSetSpeed = 0x64;  // 设定转速
constexpr uint8_t kSpeedAck = 0x65;     // 转速设定应答
constexpr uint8_t kSpeedAccTime = 0x02; // 转速指令的加速时间字段

// ---- 底盘几何与传动参数（speed_convert_rpm）----
constexpr int kLeftServoWise = 1;                                // 左电机转向
constexpr int kRightServoWise = -1;                               // 右电机转向
constexpr float kPi = 3.1415926f;
constexpr float kWheelRadius = 0.03f;                            // 轮子半径 [m]
constexpr float kWheelCircumference = 2.0f * kPi * kWheelRadius; // 轮子周长 [m]
constexpr double kWheelTread = 0.19;    // 左右轮距 [m]；沿用原宏的 double 语义以保持 rpm 结果不变
constexpr float kGearRatio = 10.0f;                               // 电机 rpm = 轮子 rpm × 减速比
constexpr float kMaxMotorRpm = 3000.0f;                           // 电机转速限幅

// ---- 速度规划与任务时序 ----
constexpr float kSecondsPerMinute = 60.0f;
constexpr float kVelocityEpsilon = 0.000001f; // 速度差小于该值即视为相等
constexpr uint32_t kPlanIntervalMs = 100;     // 两次速度规划的最小间隔
constexpr uint32_t kInitResponseWaitMs = 300; // 初始化指令等待应答
constexpr uint32_t kSpeedResponseWaitMs = 30; // 转速指令等待应答
constexpr uint32_t kMotorCmdGapMs = 5;        // 左右电机指令间隔
constexpr uint8_t kCommErrorThreshold = 3;    // 连续通信错误判定阈值

// CRC-8/MAXIM：多项式 0x31、初值 0x00、输入反写(refin=true)、输出反写(refout=true)、
// 结果异或 0x00。这里用低位在前的等价写法：每字节直接异或、右移、命中时异或反写多项式。
// 校验样例：01 A0 08 00 00 00 00 00 00 -> 0x6F，01 A0 02 00 00 00 00 00 00 -> 0xE4，
//           01 64 01 2C 00 00 00 00 00 -> 0xA6
uint8_t calc_crc8(std::span<const uint8_t> data) noexcept
{
    uint8_t crc = 0;
    for (uint8_t byte : data)
    {
        crc ^= byte;
        for (uint8_t j = 0; j < 8; ++j)
        {
            crc = (crc & 0x01) ? static_cast<uint8_t>((crc >> 1) ^ kCrcPolyReflected)
                               : static_cast<uint8_t>(crc >> 1);
        }
    }
    return crc;
}

// Arduino millis() 语义：上电以来的毫秒数，32 位自动回绕
uint32_t millis() noexcept
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// 填好最后一字节 CRC 并发送整帧（帧长固定 kFrameSize）
void send_frame(std::span<uint8_t> frame) noexcept
{
    frame.back() = calc_crc8(frame.first(frame.size() - 1));
    uart_write_bytes(kMotorUart, frame.data(), frame.size());
}

void motor_enable(MotionClass::MotorBusID id) noexcept
{
    std::array<uint8_t, kFrameSize> frame{
        static_cast<uint8_t>(id), kCmdMode,
        static_cast<uint8_t>(MotionClass::ModeCmd::SetEnable)};
    send_frame(frame);
}

void motor_set_drive_mode(MotionClass::MotorBusID id, MotionClass::ModeCmd mode) noexcept
{
    std::array<uint8_t, kFrameSize> frame{
        static_cast<uint8_t>(id), kCmdMode, static_cast<uint8_t>(mode)};
    send_frame(frame);
}

void motor_set_disconnect_stop(MotionClass::MotorBusID id) noexcept
{
    std::array<uint8_t, kFrameSize> frame{
        static_cast<uint8_t>(id), kCmdMode,
        static_cast<uint8_t>(MotionClass::ModeCmd::EnableDisConnectStop)};
    send_frame(frame);
}

void motor_set_pid(MotionClass::MotorBusID id) noexcept
{
    std::array<uint8_t, kFrameSize> frame{
        static_cast<uint8_t>(id), kCmdSetPid, 0x02, 0x28, 0x00, 0x0A, 0x00, 0x0C, 0x0A};
    send_frame(frame);
}

void motor_set_speed(MotionClass::MotorBusID id, int16_t speed) noexcept
{
    std::array<uint8_t, kFrameSize> frame{static_cast<uint8_t>(id), kCmdSetSpeed};
    frame[2] = static_cast<uint8_t>(speed >> 8);
    frame[3] = static_cast<uint8_t>(speed & 0xFF);
    frame[6] = kSpeedAccTime;
    send_frame(frame);
}

// 线速度：vel_cur/vel_set [m/min]、acc [m/min²]；角速度：rad/min、rad/min²
// interval：两次调用的时间间隔 [ms]
float velocity_planning(float vel_cur, float vel_set, float acc, uint32_t interval)
{
    acc = fabsf(acc);

    // 单精度约 7 位有效数字，速度差小于 kVelocityEpsilon 即视为相等
    if (fabsf(vel_set - vel_cur) >= kVelocityEpsilon)
    {
        // 用 copysign 给 1 赋方向判断同向/反向，避免两速度相乘溢出
        const float direction = copysignf(1, vel_cur) * copysignf(1, vel_set);
        if (direction > 0) // 当前速度与目标速度同向
        {
            acc = copysignf(acc, vel_set - vel_cur);
        }
        else if (direction < 0) // 反向
        {
            acc = copysignf(acc, vel_set);
        }
        else // 至少一个速度为 0
        {
            acc = copysignf(acc, vel_set == 0 ? -vel_cur : vel_set);
        }

        const float minutes = interval / 1000.0f / kSecondsPerMinute; // 调用间隔，单位 min
        const float change = acc * minutes;                             // 速度改变量，单位 m/min 或 rad/min
        float vel_new = vel_cur + change;
        if ((acc < 0 && vel_new < vel_set) || (acc > 0 && vel_new > vel_set)) // 加减速不能超过目标速度
        {
            vel_new = vel_set;
        }
        return vel_new;
    }
    return vel_set;
}

std::pair<int16_t, int16_t> speed_convert_rpm(float linear, float angular)
{
    // 差速：左右轮线速度 = 车体线速度 ± 角速度 × 半轮距（线速度 m/min、角速度 rad/min）
    const double half_tread = kWheelTread / 2;
    const float left_wheel_linear_speed = linear + angular * half_tread;
    const float right_wheel_linear_speed = linear - angular * half_tread;

    // 轮子转速 [rpm] = 轮线速度 / 轮子周长
    const float left_wheel_rpm = left_wheel_linear_speed / kWheelCircumference;
    const float right_wheel_rpm = right_wheel_linear_speed / kWheelCircumference;

    // 电机 rpm = 轮子 rpm × 减速比；保留原实现在 int16_t 上取整后再乘方向/限幅的行为
    int16_t left_motor_rpm = left_wheel_rpm * kGearRatio;
    int16_t right_motor_rpm = right_wheel_rpm * kGearRatio;
    left_motor_rpm *= kLeftServoWise;
    right_motor_rpm *= kRightServoWise;

    const auto clamp_rpm = [](int16_t rpm) {
        return static_cast<int16_t>(std::clamp(static_cast<float>(rpm), -kMaxMotorRpm, kMaxMotorRpm));
    };
    return {clamp_rpm(left_motor_rpm), clamp_rpm(right_motor_rpm)};
}
} // namespace

void MotionClass::begin(void)
{
    power_on();
    _rx_buffer.reserve(16);

    // ESP-IDF UART 驱动（等效 Arduino Serial1：38400 8N1，RX=17, TX=18）
    uart_config_t uart_cfg{};
    uart_cfg.baud_rate = kMotorUartBaud;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(kMotorUart, 256, 0, 10, &_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(kMotorUart, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(kMotorUart, kMotorUartTxPin, kMotorUartRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // 接收超时：RX 线上超过约 3 个字符时间（30 个波特率周期）没有新数据即认为一帧结束，
    // 这样 10 字节的响应帧会聚合成一次 UART_DATA 事件，避免逐字节拆帧
    ESP_ERROR_CHECK(uart_set_rx_timeout(kMotorUart, 30));

    xTaskCreatePinnedToCore(uart_rx_task, "motor_uart_rx", 1024 * 4, this, 2, nullptr, 0);
    xTaskCreatePinnedToCore(motion_task, "motion_task", 1024 * 10, this, 1, nullptr, 0);
}

void MotionClass::power_on(void)
{
    gpio_set_direction(static_cast<gpio_num_t>(kMotorPowerGpio), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(kMotorPowerGpio), 1);
}

void MotionClass::uart_rx_task(void *arg)
{
    uart_event_t event;
    auto *self = static_cast<MotionClass *>(arg);
    while (xQueueReceive(self->_uart_queue, &event, portMAX_DELAY))
    {
        if (event.type != UART_DATA)
        {
            continue;
        }
        uint8_t byte = 0;
        while (uart_read_bytes(kMotorUart, &byte, 1, 0) > 0)
        {
            self->_rx_buffer.emplace_back(byte);
        }
        // esplog::debug("motor rx: {:02X}", fmt::join(self->_rx_buffer, " "));
        self->_condition.notify_one();
    }
}

void MotionClass::mode_cmd_rx_data_process(MotorBusID id, ModeCmd cmd)
{
    if (_rx_buffer.size() == 10 && _rx_buffer[9] == calc_crc8(std::span(_rx_buffer).first(9)))
    {
        if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left)
        {
            _left_motor_communication_error_count = 0;
            _left_motor_communication_error = false;
        }
        else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right)
        {
            _right_motor_communication_error_count = 0;
            _right_motor_communication_error = false;
        }
        else
        {
            esplog::warn("Wrong ID.");
            return;
        }

        if (cmd == ModeCmd::SetOpenMode || cmd == ModeCmd::SetCurrentMode || cmd == ModeCmd::SetSpeedMode ||
            cmd == ModeCmd::SetPositionMode)
        {
            if (cmd == static_cast<ModeCmd>(_rx_buffer[2])) //设定值和返回值一致
            {
                if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left)
                {
                    _left_motor_mode = static_cast<DriveMode>(_rx_buffer[2]);
                }
                else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right)
                {
                    _right_motor_mode = static_cast<DriveMode>(_rx_buffer[2]);
                }
                else
                {
                    esplog::warn("Wrong ID.");
                }
            }
        }
        else if (cmd == ModeCmd::SetEnable)
        {
            if (_rx_buffer[2] == 0x01) // 1代表设定成功
            {
                if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left)
                {
                    _left_motor_state = State::enable;
                }
                else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right)
                {
                    _right_motor_state = State::enable;
                }
                else
                {
                    esplog::warn("Wrong ID.");
                }
            }
        }
        else if (cmd == ModeCmd::SetDisable)
        {
            if (_rx_buffer[2] == 0x01) // 1代表设定成功
            {
                if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left)
                {
                    _left_motor_state = State::disable;
                }
                else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right)
                {
                    _right_motor_state = State::disable;
                }
                else
                {
                    esplog::warn("Wrong ID.");
                }
            }
        }
        else if (cmd == ModeCmd::EnableDisConnectStop)
        {
            if (_rx_buffer[2] == 0x02) // 2代表设定成功
            {
                if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left)
                {
                    _left_motor_disconnect_stop = true;
                }
                else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right)
                {
                    _right_motor_disconnect_stop = true;
                }
                else
                {
                    esplog::warn("Wrong ID.");
                }
            }
        }
        else if (cmd == ModeCmd::DisableDisConnectStop)
        {
            if (_rx_buffer[2] == 0x02) // 2代表设定成功
            {
                if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left)
                {
                    _left_motor_disconnect_stop = false;
                }
                else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right)
                {
                    _right_motor_disconnect_stop = false;
                }
                else
                {
                    esplog::warn("Wrong ID.");
                }
            }
        }
        else if (cmd == ModeCmd::SetPID)
        {
            if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left &&
                _rx_buffer[1] == 0xBD && _rx_buffer[2] == 0x02)
            {
                _left_motor_pid_set = true;
            }
            else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right &&
                     _rx_buffer[1] == 0xBD && _rx_buffer[2] == 0x02)
            {
                _right_motor_pid_set = true;
            }
            else
            {
                esplog::warn("Wrong ID.");
            }
        }
    }
    else
    {
        // 连续 kCommErrorThreshold 次通信错误则认为电机通信异常
        if (id == MotorBusID::left)
        {
            esplog::warn("left motor communication error.");
            if (_left_motor_communication_error_count < kCommErrorThreshold - 1)
            {
                _left_motor_communication_error_count++;
            }
            else
            {
                _left_motor_communication_error = true;
            }
        }
        else if (id == MotorBusID::right)
        {
            esplog::warn("right motor communication error.");
            if (_right_motor_communication_error_count < kCommErrorThreshold - 1)
            {
                _right_motor_communication_error_count++;
            }
            else
            {
                _right_motor_communication_error = true;
            }
        }
    }

    _rx_buffer.clear();
}

void MotionClass::speed_set_rx_data_process(MotorBusID id)
{
    if (_rx_buffer.size() == 10 &&
        _rx_buffer[1] == kSpeedAck &&
        _rx_buffer[9] == calc_crc8(std::span(_rx_buffer).first(9)))
    {
        if (id == MotorBusID::left && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::left)
        {
            _left_motor_communication_error_count = 0;
            _left_motor_communication_error = false;
        }
        else if (id == MotorBusID::right && static_cast<MotorBusID>(_rx_buffer[0]) == MotorBusID::right)
        {
            _right_motor_communication_error_count = 0;
            _right_motor_communication_error = false;
        }
    }
    else
    {
        // 连续 kCommErrorThreshold 次通信错误则认为电机通信异常
        if (id == MotorBusID::left)
        {
            esplog::warn("left motor communication error.");
            _left_motor_communication_error_count++;
            if (_left_motor_communication_error_count == kCommErrorThreshold)
            {
                _left_motor_communication_error_count = 0;
                _left_motor_communication_error = true;
            }
        }
        else if (id == MotorBusID::right)
        {
            esplog::warn("right motor communication error.");
            _right_motor_communication_error_count++;
            if (_right_motor_communication_error_count == kCommErrorThreshold)
            {
                _right_motor_communication_error_count = 0;
                _right_motor_communication_error = true;
            }
        }
    }
    _rx_buffer.clear();
}

// 对外接口使用 SI 单位，与 geometry_msgs/Twist 一致：
//   linear  [m/s]   前进为正
//   angular [rad/s]
// 内部的速度规划、加速度标定值和 rpm 换算仍基于 m/min、rad/min
// （见 velocity_planning / speed_convert_rpm），因此这里统一乘 kSecondsPerMinute 转换一次。
void MotionClass::set_chassis_speed(float linear, float angular)
{
    _linear_speed_set = linear * kSecondsPerMinute;
    _angular_speed_set = angular * kSecondsPerMinute;
}

MotionClass::Status MotionClass::status(void) const noexcept
{
    // 内部速度单位是 m/min、rad/min，上报时统一换算成 SI（与 Twist 一致）
    constexpr float kMinToSec = 1.0f / kSecondsPerMinute;
    Status s;
    s.linear_speed_cur = _linear_speed_cur * kMinToSec;
    s.angular_speed_cur = _angular_speed_cur * kMinToSec;
    s.left_motor_rpm = _left_motor_rpm;
    s.right_motor_rpm = _right_motor_rpm;
    s.left_motor_state = static_cast<uint8_t>(_left_motor_state);
    s.right_motor_state = static_cast<uint8_t>(_right_motor_state);
    s.left_motor_mode = static_cast<uint8_t>(_left_motor_mode);
    s.right_motor_mode = static_cast<uint8_t>(_right_motor_mode);
    s.left_communication_error = _left_motor_communication_error;
    s.right_communication_error = _right_motor_communication_error;
    return s;
}

void MotionClass::motion_task(void *arg)
{
    auto self = static_cast<MotionClass *>(arg);
    std::unique_lock<std::mutex> lock(self->_mutex);
    esp_task_wdt_add(nullptr);
    while (1)
    {
        esp_task_wdt_reset();
        if (!self->_is_ready)
        {
            // 必须先使能再设置驱动模式
            if (self->_left_motor_state != State::enable)
            {
                motor_enable(MotorBusID::left);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::SetEnable);
            }
            else if (self->_left_motor_mode != DriveMode::speed)
            {
                motor_set_drive_mode(MotorBusID::left, ModeCmd::SetSpeedMode);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::SetSpeedMode);
            }
            else if (self->_left_motor_disconnect_stop != true)
            {
                motor_set_disconnect_stop(MotorBusID::left);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::EnableDisConnectStop);
            }
            else if (!self->_left_motor_pid_set)
            {
                motor_set_pid(MotorBusID::left);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::SetPID);
            }

            // 必须先使能再设置驱动模式
            if (self->_right_motor_state != State::enable)
            {
                motor_enable(MotorBusID::right);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::SetEnable);
            }
            else if (self->_right_motor_mode != DriveMode::speed)
            {
                motor_set_drive_mode(MotorBusID::right, ModeCmd::SetSpeedMode);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::SetSpeedMode);
            }
            else if (self->_right_motor_disconnect_stop != true)
            {
                motor_set_disconnect_stop(MotorBusID::right);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::EnableDisConnectStop);
            }
            else if (!self->_right_motor_pid_set)
            {
                motor_set_pid(MotorBusID::right);
                self->_condition.wait_for(lock, std::chrono::milliseconds(kInitResponseWaitMs));
                self->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::SetPID);
            }

            if (self->_left_motor_state == State::enable && self->_left_motor_mode == DriveMode::speed &&
                self->_left_motor_disconnect_stop == true && self->_left_motor_pid_set == true &&
                self->_right_motor_state == State::enable && self->_right_motor_mode == DriveMode::speed &&
                self->_right_motor_disconnect_stop == true && self->_right_motor_pid_set == true)
            {
                self->_is_ready = true;
                self->_last_plan_time = millis();
            }
        }
        else
        {
            uint32_t interval = millis() - self->_last_plan_time;
            if (interval > kPlanIntervalMs)
            {
                float linear =
                    velocity_planning(self->_linear_speed_cur, self->_linear_speed_set, self->_linear_speed_acc, interval);
                float angular =
                    velocity_planning(self->_angular_speed_cur, self->_angular_speed_set, self->_angular_speed_acc, interval);
                // float linear = m->_linear_speed_set;
                // float angular = m->_angular_speed_set;
                auto [left_motor_rpm, right_motor_rpm] = speed_convert_rpm(linear, angular);
                self->_linear_speed_cur = linear;
                self->_angular_speed_cur = angular;
                self->_left_motor_rpm = left_motor_rpm;
                self->_right_motor_rpm = right_motor_rpm;
                self->_last_plan_time = millis();
            }

            motor_set_speed(MotionClass::MotorBusID::left, self->_left_motor_rpm);
            self->_condition.wait_for(lock, std::chrono::milliseconds(kSpeedResponseWaitMs));
            self->speed_set_rx_data_process(MotionClass::MotorBusID::left);
            vTaskDelay(pdMS_TO_TICKS(kMotorCmdGapMs));
            motor_set_speed(MotionClass::MotorBusID::right, self->_right_motor_rpm);
            self->_condition.wait_for(lock, std::chrono::milliseconds(kSpeedResponseWaitMs));
            self->speed_set_rx_data_process(MotionClass::MotorBusID::right);
            vTaskDelay(pdMS_TO_TICKS(kMotorCmdGapMs));
        }
    }
}

std::ostream &operator<<(std::ostream &os, const MotionClass &m)
{
    os << "Left motor state/mode/disconnect_stop:" << m._left_motor_state << "/" << m._left_motor_mode << "/"
       << m._left_motor_disconnect_stop << std::endl;
    os << "Right motor state/mode/disconnect_stop:" << m._right_motor_state << "/" << m._right_motor_mode << "/"
       << m._right_motor_disconnect_stop << std::endl;
    return os;
}
