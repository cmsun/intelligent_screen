#pragma once

// 自定义串口链路协议（ESP32 <-> OrangePi）。
// 本头文件为两端共享，禁止包含 ESP-IDF / ROS / 平台专用头文件，保证固件与上位机
// 的帧格式、消息 ID、payload 布局、CRC 算法完全一致。
//
// 帧格式（小端）：A5 5A | msg_id(u8) | len(u8) | payload[len] | crc8(u8)
// CRC8（多项式 0x8C，初值 0x00）覆盖 msg_id + len + payload。
// 帧头字节出现在 payload 中不会造成歧义：解析器按长度字段收满后才校验 CRC。

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace chassis_proto
{
constexpr uint8_t kFrameHead0 = 0xA5;
constexpr uint8_t kFrameHead1 = 0x5A;
constexpr uint8_t kMaxPayload = 64;
constexpr uint32_t kBaudRate = 115200;

enum MsgId : uint8_t
{
    // 上位机 -> ESP32（命令）
    kCmdVel = 0x01,            // VelCmd：线速度 m/s、角速度 rad/s
    kCmdServo = 0x02,          // ServoCmd：目标角度 deg、转动时间 ms
    kCmdBatteryOffset = 0x03,  // BatteryOffsetCmd：电流零漂 A，写入 NVS
    kCmdPing = 0x04,           // 无 payload，链路保活

    // ESP32 -> 上位机（周期状态）
    kStatusImu = 0x10,         // ImuStatus，10Hz
    kStatusServo = 0x11,       // ServoStatus，10Hz（电压 0.5Hz）
    kStatusXl9535 = 0x12,      // Xl9535Status，10Hz
    kStatusIr = 0x13,          // IrStatus，10Hz
    kStatusMotion = 0x14,      // MotionStatus，10Hz
    kStatusBattery = 0x15,     // BatteryStatus，1Hz
    kStatusPong = 0x16,        // 无 payload，Ping 应答
};

// Xl9535Status.flags
constexpr uint8_t kXlFloorLeft = 1u << 0;
constexpr uint8_t kXlFloorLeftMiddle = 1u << 1;
constexpr uint8_t kXlFloorRightMiddle = 1u << 2;
constexpr uint8_t kXlFloorRight = 1u << 3;
constexpr uint8_t kXlHeadUpObstacle = 1u << 4;
constexpr uint8_t kXlHeadDownObstacle = 1u << 5;
constexpr uint8_t kXlFanOn = 1u << 6;
constexpr uint8_t kXlIntActive = 1u << 7;

// IrStatus.flags
constexpr uint8_t kIrAddressValid = 1u << 0;
constexpr uint8_t kIrRepeat = 1u << 1;
constexpr uint8_t kIrLeftActive = 1u << 2;
constexpr uint8_t kIrRightActive = 1u << 3;

// MotionStatus.comm_errors
constexpr uint8_t kMotionLeftCommError = 1u << 0;
constexpr uint8_t kMotionRightCommError = 1u << 1;

// BatteryStatus.flags
constexpr uint8_t kBatteryChargeEnabled = 1u << 0;

// BatteryStatus.state，与 BatteryClass::State 一一对应
enum BatteryState : uint8_t
{
    kBatteryIdle = 0,
    kBatteryDischarging = 1,
    kBatteryCharging = 2,
    kBatteryFull = 3,
    kBatteryLow = 4,
    kBatteryOverCurrent = 5,
};

#pragma pack(push, 1)
struct VelCmd
{
    float linear_mps;
    float angular_rps;
};
struct ServoCmd
{
    float angle_deg;
    uint16_t move_time_ms;
};
struct BatteryOffsetCmd
{
    float offset_a;
};

struct ImuStatus
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_rate_deg_s;
};
struct ServoStatus
{
    uint8_t ready;  // 已成功读到过位置即保持为 1（握手语义）
    uint8_t id;
    int32_t position;
    float angle_deg;
    float voltage_v;
};
struct Xl9535Status
{
    uint8_t flags;
};
struct IrStatus
{
    uint8_t side;  // 0=左 1=右
    uint8_t address;
    uint8_t command;
    uint32_t raw_code;
    uint8_t flags;
    uint32_t timestamp_ms;
};
struct MotionStatus
{
    float linear_speed_cur;
    float angular_speed_cur;
    int16_t left_motor_rpm;
    int16_t right_motor_rpm;
    uint8_t left_motor_state;
    uint8_t right_motor_state;
    uint8_t left_motor_mode;
    uint8_t right_motor_mode;
    uint8_t comm_errors;
};
struct BatteryStatus
{
    float battery_voltage;
    float charger_voltage;
    float current;
    float power;
    float soc;
    uint8_t state;
    uint8_t flags;
    float current_offset;
};
#pragma pack(pop)

inline uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1u) ? static_cast<uint8_t>((crc >> 1) ^ 0x8C) : static_cast<uint8_t>(crc >> 1);
        }
    }
    return crc;
}

// 组帧，返回总字节数；buf 至少要有 kMaxPayload + 5 字节
inline size_t build_frame(uint8_t *buf, uint8_t msg_id, const void *payload, uint8_t len)
{
    buf[0] = kFrameHead0;
    buf[1] = kFrameHead1;
    buf[2] = msg_id;
    buf[3] = len;
    if (len > 0 && payload != nullptr)
    {
        std::memcpy(&buf[4], payload, len);
    }
    buf[4 + len] = crc8(&buf[2], static_cast<size_t>(2) + len);
    return static_cast<size_t>(5) + len;
}

// 逐字节帧解析器：两端共用。feed() 返回非 0 表示收到一帧有效消息，
// payload 拷入 out（最多 out_cap 字节），长度写入 out_len；返回 0 表示尚未成帧或 CRC 失败。
class FrameParser
{
public:
    uint8_t feed(uint8_t byte, uint8_t *out, uint8_t out_cap, uint8_t &out_len)
    {
        out_len = 0;
        switch (_state)
        {
        case kWaitHead0:
            if (byte == kFrameHead0)
            {
                _state = kWaitHead1;
            }
            break;
        case kWaitHead1:
            if (byte == kFrameHead1)
            {
                _state = kWaitId;
            }
            else
            {
                _state = (byte == kFrameHead0) ? kWaitHead1 : kWaitHead0;
            }
            break;
        case kWaitId:
            _id = byte;
            _buf[0] = byte;
            _state = kWaitLen;
            break;
        case kWaitLen:
            if (byte > kMaxPayload)
            {
                _state = kWaitHead0;
                break;
            }
            _len = byte;
            _buf[1] = byte;
            _pos = 0;
            _state = (byte == 0) ? kWaitCrc : kWaitPayload;
            break;
        case kWaitPayload:
            _buf[2 + _pos++] = byte;
            if (_pos >= _len)
            {
                _state = kWaitCrc;
            }
            break;
        case kWaitCrc:
        {
            const uint8_t id = _id;
            const uint8_t len = _len;
            const bool ok = (byte == crc8(_buf, static_cast<size_t>(2) + len));
            _state = kWaitHead0;
            if (!ok)
            {
                break;
            }
            out_len = (len <= out_cap) ? len : out_cap;
            if (out_len > 0 && out != nullptr)
            {
                std::memcpy(out, &_buf[2], out_len);
            }
            return id;
        }
        }
        return 0;
    }

private:
    enum State
    {
        kWaitHead0,
        kWaitHead1,
        kWaitId,
        kWaitLen,
        kWaitPayload,
        kWaitCrc,
    };
    State _state = kWaitHead0;
    uint8_t _id = 0;
    uint8_t _len = 0;
    uint8_t _pos = 0;
    uint8_t _buf[2 + kMaxPayload] = {};  // id + len + payload，便于一次算 CRC
};
} // namespace chassis_proto
