#include "serial_comm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <driver/uart.h>
#include <esp_err.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "battery.hpp"
#include "bus_servo.hpp"
#include "esplog.hpp"
#include "ir_receiver.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"
#include "xl9535.hpp"

namespace
{
// 与 master 分支一致的限幅与周期参数
constexpr float kMaxLinearMps = 0.6f;
constexpr float kMaxAngularRps = 2.0f;
constexpr int64_t kVelcmdTimeoutUs = 500LL * 1000;
constexpr int64_t kStatusPublishPeriodUs = 100LL * 1000;
constexpr int64_t kBatteryPublishPeriodUs = 1000LL * 1000;
constexpr int64_t kServoVoltagePeriodUs = 2000LL * 1000;
// 上位机静止时靠 PING（500ms）保活，2s 无有效帧即按链路断开处理
constexpr int64_t kLinkTimeoutUs = 2000LL * 1000;

// ESP32-S3 UART0 默认 GPIO43(TXD)/GPIO44(RXD)；console 已切到 USB Serial/JTAG
constexpr uart_port_t kUartPort = UART_NUM_0;
constexpr int kUartTxdPin = 43;
constexpr int kUartRxdPin = 44;
constexpr int kUartRxBufferBytes = 1024;
constexpr int kUartTxBufferBytes = 1024;

// 地检防跌落：低电平有效（0 = 有地面 detected，1 = 悬空 lost），
// floor_lost_mask() 中置 1 表示该路悬空（触发保护）。
// 逐分量钳制，只把违规分量置 0，合法分量原样保留：
//   任一路触发  : linear  > 0 -> 0
//   左/左中触发 : angular > 0 -> 0
//   右/右中触发 : angular < 0 -> 0
// 左右两侧同时触发时两条角速度规则叠加，angular 恒为 0。
void floor_guard_clamp(float &linear, float &angular, uint8_t &lost_mask)
{
    lost_mask = Xl9535.floor_lost_mask();
    if (lost_mask == 0)
    {
        return;
    }
    if (linear > 0.0f)
    {
        linear = 0.0f;
    }

    constexpr uint8_t kLeftSide = static_cast<uint8_t>(Xl9535Class::kFloorLostLeft |
                                                        Xl9535Class::kFloorLostLeftMiddle);
    constexpr uint8_t kRightSide = static_cast<uint8_t>(Xl9535Class::kFloorLostRightMiddle |
                                                         Xl9535Class::kFloorLostRight);
    if ((lost_mask & kLeftSide) != 0 && angular > 0.0f)
    {
        angular = 0.0f;
    }
    if ((lost_mask & kRightSide) != 0 && angular < 0.0f)
    {
        angular = 0.0f;
    }
}

uint8_t xl9535_flags(void)
{
    uint8_t flags = 0;
    if (Xl9535.floor_detected(0)) flags |= chassis_proto::kXlFloorLeft;
    if (Xl9535.floor_detected(1)) flags |= chassis_proto::kXlFloorLeftMiddle;
    if (Xl9535.floor_detected(2)) flags |= chassis_proto::kXlFloorRightMiddle;
    if (Xl9535.floor_detected(3)) flags |= chassis_proto::kXlFloorRight;
    if (Xl9535.head_up_obstacle()) flags |= chassis_proto::kXlHeadUpObstacle;
    if (Xl9535.head_down_obstacle()) flags |= chassis_proto::kXlHeadDownObstacle;
    if (Xl9535.fan_on()) flags |= chassis_proto::kXlFanOn;
    if (Xl9535.int_active()) flags |= chassis_proto::kXlIntActive;
    return flags;
}

uint8_t ir_flags(const IrFrame &frame, bool left_active, bool right_active)
{
    uint8_t flags = 0;
    if (frame.address_valid) flags |= chassis_proto::kIrAddressValid;
    if (frame.repeat) flags |= chassis_proto::kIrRepeat;
    if (left_active) flags |= chassis_proto::kIrLeftActive;
    if (right_active) flags |= chassis_proto::kIrRightActive;
    return flags;
}
} // namespace

SerialCommClass SerialComm;

void SerialCommClass::begin(void)
{
    // 先零初始化再用字段赋值：uart_config_t 还有 rx_flow_ctrl_thresh / flags 等字段，
    // 用 designated initializer 会触发 -Wmissing-field-initializers
    uart_config_t cfg = {};
    cfg.baud_rate = static_cast<int>(chassis_proto::kBaudRate);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(kUartPort, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(kUartPort, kUartTxdPin, kUartRxdPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(kUartPort, kUartRxBufferBytes, kUartTxBufferBytes, 0, nullptr, 0));
    esplog::info("custom serial link ready: UART0 GPIO{}/GPIO{}, {} baud", kUartTxdPin, kUartRxdPin,
                 static_cast<int>(chassis_proto::kBaudRate));

    xTaskCreatePinnedToCore(rx_task, "serial_rx", 1024 * 4, this, 3, nullptr, 1);
    xTaskCreatePinnedToCore(status_task, "serial_status", 1024 * 4, this, 2, nullptr, 1);
}

void SerialCommClass::send(uint8_t msg_id, const void *payload, uint8_t len)
{
    uint8_t buf[chassis_proto::kMaxPayload + 5] = {};
    const size_t n = chassis_proto::build_frame(buf, msg_id, payload, len);
    uart_write_bytes(kUartPort, reinterpret_cast<const char *>(buf), n);
}

void SerialCommClass::rx_task(void *arg)
{
    esp_task_wdt_add(nullptr);
    auto *self = static_cast<SerialCommClass *>(arg);
    chassis_proto::FrameParser parser;
    uint8_t payload[chassis_proto::kMaxPayload] = {};
    uint8_t payload_len = 0;
    uint8_t raw[64] = {};

    while (true)
    {
        esp_task_wdt_reset();
        const int n = uart_read_bytes(kUartPort, raw, sizeof(raw), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; ++i)
        {
            const uint8_t msg_id = parser.feed(raw[i], payload, sizeof(payload), payload_len);
            if (msg_id == 0)
            {
                continue;
            }
            self->_last_frame_us.store(esp_timer_get_time(), std::memory_order_release);
            self->handle_frame(msg_id, payload, payload_len);
        }
    }
}

void SerialCommClass::handle_frame(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    switch (msg_id)
    {
    case chassis_proto::kCmdVel:
    {
        if (len != sizeof(chassis_proto::VelCmd)) break;
        chassis_proto::VelCmd cmd{};
        std::memcpy(&cmd, payload, sizeof(cmd));
        apply_velcmd(cmd.linear_mps, cmd.angular_rps);
        break;
    }
    case chassis_proto::kCmdServo:
    {
        if (len != sizeof(chassis_proto::ServoCmd)) break;
        chassis_proto::ServoCmd cmd{};
        std::memcpy(&cmd, payload, sizeof(cmd));
        if (!BusServo.is_ready()) break;  // 总线未就绪时静默忽略，与 master 一致
        if (!BusServo.move_to_angle(cmd.angle_deg, cmd.move_time_ms))
        {
            esplog::warn("bus servo move to {} deg in {} ms failed", cmd.angle_deg, cmd.move_time_ms);
        }
        break;
    }
    case chassis_proto::kCmdBatteryOffset:
    {
        if (len != sizeof(chassis_proto::BatteryOffsetCmd)) break;
        chassis_proto::BatteryOffsetCmd cmd{};
        std::memcpy(&cmd, payload, sizeof(cmd));
        if (!Battery.set_current_offset(cmd.offset_a))
        {
            esplog::error("set battery current offset {} A failed", cmd.offset_a);
            break;
        }
        esplog::info("battery current offset set to {} A (saved to NVS)", cmd.offset_a);
        break;
    }
    case chassis_proto::kCmdPing:
        send(chassis_proto::kStatusPong, nullptr, 0);
        break;
    default:
        break;
    }
}

void SerialCommClass::apply_velcmd(float linear, float angular)
{
    // MotionClass::set_chassis_speed() 同为 SI 单位，这里只做限幅 + 地检防跌落钳制
    linear = std::clamp(linear, -kMaxLinearMps, kMaxLinearMps);
    angular = std::clamp(angular, -kMaxAngularRps, kMaxAngularRps);
    const float raw_linear = linear;
    const float raw_angular = angular;

    uint8_t lost_mask = 0;
    floor_guard_clamp(linear, angular, lost_mask);
    if (!std::isfinite(linear) || !std::isfinite(angular))
    {
        linear = 0.0f;
        angular = 0.0f;
    }

    if (linear != raw_linear || angular != raw_angular)
    {
        // 钳制日志按 1s 节流，避免 20Hz 指令刷屏
        static int64_t last_clamp_log_us = 0;
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_clamp_log_us >= 1000LL * 1000)
        {
            last_clamp_log_us = now_us;
            esplog::warn("floor guard clamp (lost 0x{:02X}): ({}, {}) -> ({}, {})",
                         lost_mask, raw_linear, raw_angular, linear, angular);
        }
    }

    Motion.set_chassis_speed(linear, angular);
    _last_velcmd_us.store(esp_timer_get_time(), std::memory_order_release);
}

void SerialCommClass::status_task(void *arg)
{
    esp_task_wdt_add(nullptr);
    auto *self = static_cast<SerialCommClass *>(arg);
    int64_t last_status_us = 0;
    int64_t last_battery_us = 0;
    int64_t last_servo_voltage_us = 0;

    while (true)
    {
        esp_task_wdt_reset();
        const int64_t now_us = esp_timer_get_time();

        // 链路断开检测：对应 master 分支的 micro-ROS agent 断链停车
        const int64_t last_frame_us = self->_last_frame_us.load(std::memory_order_acquire);
        const bool connected = (last_frame_us != 0 && now_us - last_frame_us <= kLinkTimeoutUs);
        if (!connected && self->_link_connected)
        {
            Motion.set_chassis_speed(0.0f, 0.0f);
            self->_last_velcmd_us.store(0, std::memory_order_release);
            esplog::warn("serial link lost, chassis stopped");
        }
        self->_link_connected = connected;

        if (now_us - last_status_us >= kStatusPublishPeriodUs)
        {
            last_status_us = now_us;

            chassis_proto::ImuStatus imu{};
            if (Imu.is_ready())
            {
                const Vec3 euler = Imu.euler_deg();
                imu.roll_deg = euler.x;
                imu.pitch_deg = euler.y;
                imu.yaw_deg = euler.z;
                imu.yaw_rate_deg_s = Imu.yaw_rate_deg_s();
            }
            self->send(chassis_proto::kStatusImu, &imu, sizeof(imu));

            chassis_proto::Xl9535Status xl{};
            xl.flags = xl9535_flags();
            self->send(chassis_proto::kStatusXl9535, &xl, sizeof(xl));

            const IrFrame frame = IrReceiver.last_frame();
            chassis_proto::IrStatus ir{};
            ir.side = static_cast<uint8_t>(frame.side == IrSide::Right ? 1 : 0);
            ir.address = frame.address;
            ir.command = frame.command;
            ir.raw_code = frame.raw_code;
            ir.flags = ir_flags(frame, IrReceiver.signal_seen(IrSide::Left, 500),
                                IrReceiver.signal_seen(IrSide::Right, 500));
            ir.timestamp_ms = frame.timestamp_ms;
            self->send(chassis_proto::kStatusIr, &ir, sizeof(ir));

            const MotionClass::Status motion = Motion.status();
            chassis_proto::MotionStatus mot{};
            mot.linear_speed_cur = motion.linear_speed_cur;
            mot.angular_speed_cur = motion.angular_speed_cur;
            mot.left_motor_rpm = motion.left_motor_rpm;
            mot.right_motor_rpm = motion.right_motor_rpm;
            mot.left_motor_state = motion.left_motor_state;
            mot.right_motor_state = motion.right_motor_state;
            mot.left_motor_mode = motion.left_motor_mode;
            mot.right_motor_mode = motion.right_motor_mode;
            if (motion.left_communication_error) mot.comm_errors |= chassis_proto::kMotionLeftCommError;
            if (motion.right_communication_error) mot.comm_errors |= chassis_proto::kMotionRightCommError;
            self->send(chassis_proto::kStatusMotion, &mot, sizeof(mot));

            // 舵机：10Hz 读位置（成功过即 ready 保持 true），2s 读一次电压
            uint16_t position = 0;
            if (BusServo.is_ready() && BusServo.read_position(position))
            {
                self->_servo_ready = 1;
                self->_servo_id = BusServo.id();
                self->_servo_position = static_cast<int32_t>(position);
                self->_servo_angle_deg = BusServoClass::position_to_angle(position);
            }
            if (now_us - last_servo_voltage_us >= kServoVoltagePeriodUs)
            {
                last_servo_voltage_us = now_us;
                uint16_t voltage_mv = 0;
                if (BusServo.is_ready() && BusServo.read_voltage_mv(voltage_mv))
                {
                    self->_servo_voltage_v = static_cast<float>(voltage_mv) / 1000.0f;
                }
            }
            chassis_proto::ServoStatus servo{};
            servo.ready = self->_servo_ready;
            servo.id = self->_servo_id;
            servo.position = self->_servo_position;
            servo.angle_deg = self->_servo_angle_deg;
            servo.voltage_v = self->_servo_voltage_v;
            self->send(chassis_proto::kStatusServo, &servo, sizeof(servo));
        }

        if (now_us - last_battery_us >= kBatteryPublishPeriodUs)
        {
            last_battery_us = now_us;
            const BatteryClass::Status battery = Battery.status();
            chassis_proto::BatteryStatus bat{};
            bat.battery_voltage = battery.battery_voltage;
            bat.charger_voltage = battery.charger_voltage;
            bat.current = battery.current;
            bat.power = battery.power;
            bat.soc = battery.soc;
            bat.state = static_cast<uint8_t>(battery.state);
            if (battery.charge_enabled) bat.flags |= chassis_proto::kBatteryChargeEnabled;
            // 回读本板当前生效的零漂值，供上位机标定流程判断
            bat.current_offset = Battery.current_offset();
            self->send(chassis_proto::kStatusBattery, &bat, sizeof(bat));
        }

        // 手柄断连 / 上位机卡住时，不能让底盘继续跑最后一条速度指令
        const int64_t last_cmd_us = self->_last_velcmd_us.load(std::memory_order_acquire);
        if (last_cmd_us != 0 && now_us - last_cmd_us > kVelcmdTimeoutUs)
        {
            if (!self->_velcmd_timed_out)
            {
                Motion.set_chassis_speed(0.0f, 0.0f);
                self->_velcmd_timed_out = true;
                esplog::warn("velcmd timeout, chassis stopped");
            }
        }
        else
        {
            self->_velcmd_timed_out = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
