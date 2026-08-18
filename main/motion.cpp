#include "motion.hpp"

#include "esplog.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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
constexpr uart_port_t kMotorUart = UART_NUM_1;
constexpr int kMotorUartTxPin = 18; // 电机控制板 UART TX（原 Arduino Serial1 TX）
constexpr int kMotorUartRxPin = 17; // 电机控制板 UART RX（原 Arduino Serial1 RX）
constexpr uint32_t kMotorUartBaud = 38400;
constexpr int kMotorPowerGpio = 39; // 电机电源使能引脚（原 pinMode(39, OUTPUT)）

constexpr uint8_t kCrcPoly = 0x31;

uint8_t reverse8(uint8_t v) noexcept
{
    v = static_cast<uint8_t>(((v & 0x55) << 1) | ((v & 0xAA) >> 1));
    v = static_cast<uint8_t>(((v & 0x33) << 2) | ((v & 0xCC) >> 2));
    return static_cast<uint8_t>((v << 4) | (v >> 4));
}

// 与原 Arduino CRC 库 calcCRC8(data, len, 0x31, 0, 0, true, true) 逐位一致：
// 多项式 0x31、初始值 0、输入不反写(refin=false)、输出反写(refout=true)、最后异或 1
uint8_t calc_crc8(const uint8_t data[], uint8_t len) noexcept
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j)
        {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ kCrcPoly)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    crc = reverse8(crc);
    crc ^= 0x01;
    return crc;
}

// Arduino millis() 语义：上电以来的毫秒数，32 位自动回绕
uint32_t millis() noexcept
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
} // namespace

static inline void motor_enable(MotionClass::MotorBusID id)
{
    uint8_t buff[10] = {uint8_t(id), 0xA0, uint8_t(MotionClass::ModeCmd::SetEnable)};
    uint8_t crc = calc_crc8(buff, sizeof(buff) - 1);
    buff[sizeof(buff) - 1] = crc;
    uart_write_bytes(kMotorUart, buff, sizeof(buff));
}

static inline void motor_set_drive_mode(MotionClass::MotorBusID id, MotionClass::ModeCmd mode)
{
    uint8_t buff[10] = {uint8_t(id), 0xA0, uint8_t(mode)};
    uint8_t crc = calc_crc8(buff, sizeof(buff) - 1);
    buff[sizeof(buff) - 1] = crc;
    uart_write_bytes(kMotorUart, buff, sizeof(buff));
}

static inline void motor_set_disconnect_stop(MotionClass::MotorBusID id)
{
    uint8_t buff[10] = {uint8_t(id), 0xA0, uint8_t(MotionClass::ModeCmd::EnableDisConnectStop)};
    uint8_t crc = calc_crc8(buff, sizeof(buff) - 1);
    buff[sizeof(buff) - 1] = crc;
    uart_write_bytes(kMotorUart, buff, sizeof(buff));
}

static inline void motor_set_pid(MotionClass::MotorBusID id)
{
    uint8_t buff[10] = {uint8_t(id), 0xBD, 0x02, 0x28, 0x00, 0x0A, 0x00, 0x0C, 0x0A};
    uint8_t crc = calc_crc8(buff, sizeof(buff) - 1);
    buff[sizeof(buff) - 1] = crc;
    uart_write_bytes(kMotorUart, buff, sizeof(buff));
}

static inline void motor_set_speed(MotionClass::MotorBusID id, int16_t speed)
{
    uint8_t buff[10] = {uint8_t(id), 0x64};
    buff[2] = speed >> 8;
    buff[3] = speed & 0xFF;
    buff[6] = 0x02;
    uint8_t crc = calc_crc8(buff, sizeof(buff) - 1);
    buff[sizeof(buff) - 1] = crc;
    uart_write_bytes(kMotorUart, buff, sizeof(buff));
}

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
    auto *m = static_cast<MotionClass *>(arg);
    uart_event_t event;
    while (xQueueReceive(m->_uart_queue, &event, portMAX_DELAY))
    {
        if (event.type != UART_DATA)
        {
            continue;
        }
        uint8_t byte = 0;
        while (uart_read_bytes(kMotorUart, &byte, 1, 0) > 0)
        {
            m->_rx_buffer.emplace_back(byte);
        }
        m->_condition.notify_one();
    }
}

void MotionClass::mode_cmd_rx_data_process(MotorBusID id, ModeCmd cmd)
{
    if (_rx_buffer.size() == 10 && _rx_buffer[9] == calc_crc8(_rx_buffer.data(), 9))
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
            esplog::warn("Wrong sequence.");
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
                    esplog::warn("Wrong sequence.");
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
                    esplog::warn("Wrong sequence.");
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
                    esplog::warn("Wrong sequence.");
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
                    esplog::warn("Wrong sequence.");
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
                    esplog::warn("Wrong sequence.");
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
                esplog::warn("Wrong sequence.");
            }
        }
    }
    else
    {
        // 通信错误计数，连续3次通信错误则认为电机通信异常
        if (id == MotorBusID::left)
        {
            esplog::warn("left motor communication error.");
            if (_left_motor_communication_error_count < 2)
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
            if (_right_motor_communication_error_count < 2)
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
    if (_rx_buffer.size() == 10 && _rx_buffer[9] == calc_crc8(_rx_buffer.data(), 9))
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
        // 通信错误计数，连续3次通信错误则认为电机通信异常
        if (id == MotorBusID::left)
        {
            esplog::warn("left motor communication error.");
            if (_left_motor_communication_error_count < 2)
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
            if (_right_motor_communication_error_count < 2)
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

//linear speed and angular speed convert motor RPM.
//linear: linear speed of chassis, unit: m/min
//angular: angular speed of chassis, unit: rad/min
void MotionClass::set_chassis_speed(float linear, float angular)
{
    _linear_speed_set = linear;
    _angular_speed_set = angular;
}

//如果是线速度，vel_cur和vel_set单位为m/min，加速度为m/min^2
//如果是角速度，vel_cur和vel_set单位为rad/min，加速度为rad/min^2
//interval：两次调用函数的时间间隔，单位为ms
float MotionClass::velocity_planning(float vel_cur, float vel_set, float acc, uint32_t interval)
{
    float direction, minutes, vel_new, change;

    acc = fabsf(acc);

    //单精度浮点数能表示7位有效数字(整数部分加小数部分)。
    //所以当目标速度和当前速度之差大于等于0.000001f时认为两个速度不相等，速度之差小于0.000001f时才认为速度相等。
    if (fabsf(vel_set - vel_cur) >= 0.000001f)
    {
        direction = copysignf(1, vel_cur) *
                    copysignf(1, vel_set); //判断两个速度是同向还是反向，用copysign给1赋值方向，防止相乘溢出
        if (direction > 0)                 //当前速度和目标速度同向
        {
            acc = copysignf(acc, vel_set - vel_cur);
        }
        else if (direction < 0) //当前速度和目标速度反向
        {
            acc = copysignf(acc, vel_set);
        }
        else
        {
            if (vel_set == 0)
            {
                acc = copysignf(acc, -vel_cur);
            }
            else
            {
                acc = copysignf(acc, vel_set);
            }
        }

        minutes = interval / 1000.f / 60.f;                                   //两次函数调用的时间间隔 UNIT：min
        change = acc * minutes;                                               //根据加速度计算速度改变量 UNIT:m/s
        vel_new = vel_cur + change;                                           //上个周期速度加上速度改变量 UNIT:m/min
        if ((acc < 0 && vel_new < vel_set) || (acc > 0 && vel_new > vel_set)) //加减速后不能超过目标速度
        {
            vel_new = vel_set;
        }
    }
    else
    {
        vel_new = vel_set;
    }

    return vel_new;
}

std::pair<int16_t, int16_t> MotionClass::speed_convert_rpm(float linear, float angular)
{
#define LEFT_SERVO_WISE (1)
#define RIGHT_SERVO_WISE (-1)
#define WHEEL_RADIUS 0.03f                            //轮子半径 单位(m)
#define WHEEL_CICRUMFERENCE (2 * 3.1415926f * WHEEL_RADIUS) //轮子周长 单位(m)
#define WHEEL_TREAD 0.3725f                           //轮距 单位(m)

    float left_wheel_linear_speed, right_wheel_linear_speed;
    float left_wheel_rpm, right_wheel_rpm;
    int16_t left_motor_rpm, right_motor_rpm;

    left_wheel_linear_speed = linear + angular * (WHEEL_TREAD / 2);  //左边轮子的线速度 UNIT:m/min
    right_wheel_linear_speed = linear - angular * (WHEEL_TREAD / 2); //右边轮子的线速度 UNIT:m/min

    left_wheel_rpm = left_wheel_linear_speed / WHEEL_CICRUMFERENCE;   //左边轮子的转速
    right_wheel_rpm = right_wheel_linear_speed / WHEEL_CICRUMFERENCE; //右边轮子的转速
    //左边轮子的转速乘以减速比等于左边伺服电机转速
    left_motor_rpm = left_wheel_rpm * 10;
    //右边轮子的转速乘以减速比等于右边伺服电机转速
    right_motor_rpm = right_wheel_rpm * 10;

    left_motor_rpm *= LEFT_SERVO_WISE;   //设置左边伺服电机转向
    right_motor_rpm *= RIGHT_SERVO_WISE; //设置右边伺服电机转向

    // 伺服电机的最高转速为300转
    left_motor_rpm = static_cast<int16_t>(std::clamp(static_cast<float>(left_motor_rpm), -3000.0f, +3000.0f));
    //伺服电机的最高转速为300转
    right_motor_rpm = static_cast<int16_t>(std::clamp(static_cast<float>(right_motor_rpm), -3000.0f, +3000.0f));

    return std::make_pair(left_motor_rpm, right_motor_rpm);
}

void MotionClass::motion_task(void *arg)
{
    auto m = static_cast<MotionClass *>(arg);
    std::unique_lock<std::mutex> lock(m->_mutex);
    esp_task_wdt_add(nullptr);
    while (1)
    {
        esp_task_wdt_reset();
        if (!m->_is_ready)
        {
            // 必须先使能再设置驱动模式
            if (m->_left_motor_state != State::enable)
            {
                motor_enable(MotorBusID::left);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::SetEnable);
            }
            else if (m->_left_motor_mode != DriveMode::speed)
            {
                motor_set_drive_mode(MotorBusID::left, ModeCmd::SetSpeedMode);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::SetSpeedMode);
            }
            else if (m->_left_motor_disconnect_stop != true)
            {
                motor_set_disconnect_stop(MotorBusID::left);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::EnableDisConnectStop);
            }
            else if (!m->_left_motor_pid_set)
            {
                motor_set_pid(MotorBusID::left);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::left, ModeCmd::SetPID);
            }

            // 必须先使能再设置驱动模式
            if (m->_right_motor_state != State::enable)
            {
                motor_enable(MotorBusID::right);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::SetEnable);
            }
            else if (m->_right_motor_mode != DriveMode::speed)
            {
                motor_set_drive_mode(MotorBusID::right, ModeCmd::SetSpeedMode);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::SetSpeedMode);
            }
            else if (m->_right_motor_disconnect_stop != true)
            {
                motor_set_disconnect_stop(MotorBusID::right);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::EnableDisConnectStop);
            }
            else if (!m->_right_motor_pid_set)
            {
                motor_set_pid(MotorBusID::right);
                m->_condition.wait_for(lock, std::chrono::milliseconds(300));
                m->mode_cmd_rx_data_process(MotorBusID::right, ModeCmd::SetPID);
            }

            if (m->_left_motor_state == State::enable && m->_left_motor_mode == DriveMode::speed &&
                m->_left_motor_disconnect_stop == true && m->_left_motor_pid_set == true &&
                m->_right_motor_state == State::enable && m->_right_motor_mode == DriveMode::speed &&
                m->_right_motor_disconnect_stop == true && m->_right_motor_pid_set == true)
            {
                m->_is_ready = true;
                m->_last_plan_time = millis();
            }
        }
        else
        {
            uint32_t interval = millis() - m->_last_plan_time;
            if (interval > 100)
            {
                float linear =
                    velocity_planning(m->_linear_speed_cur, m->_linear_speed_set, m->_linear_speed_acc, interval);
                float angular =
                    velocity_planning(m->_angular_speed_cur, m->_angular_speed_set, m->_angular_speed_acc, interval);
                // float linear = m->_linear_speed_set;
                // float angular = m->_angular_speed_set;
                auto [left_motor_rpm, right_motor_rpm] = speed_convert_rpm(linear, angular);
                m->_linear_speed_cur = linear;
                m->_angular_speed_cur = angular;
                m->_left_motor_rpm = left_motor_rpm;
                m->_right_motor_rpm = right_motor_rpm;
                m->_last_plan_time = millis();
            }

            motor_set_speed(MotionClass::MotorBusID::left, m->_left_motor_rpm);
            m->_condition.wait_for(lock, std::chrono::milliseconds(300));
            m->speed_set_rx_data_process(MotionClass::MotorBusID::left);
            vTaskDelay(pdMS_TO_TICKS(5));
            motor_set_speed(MotionClass::MotorBusID::right, m->_right_motor_rpm);
            m->_condition.wait_for(lock, std::chrono::milliseconds(300));
            m->speed_set_rx_data_process(MotionClass::MotorBusID::right);
            vTaskDelay(pdMS_TO_TICKS(5));
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
