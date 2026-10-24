#pragma once

#include <cstdint>

///
/// 电机 ID 设置工具（出厂/维护时一次性使用，正常运行的固件不要调用）
///
/// 使用步骤：
///   1. 在 main.cpp 里把 Motion.begin(); 注释掉。
///      Motion 启动后会独占 UART1（TX=18 / RX=17）并周期性下发使能、驱动模式、
///      速度等指令，会与本工具的设置 ID 指令及其反馈混在同一条总线上，
///      导致反馈解析出错、甚至把指令互相覆盖。
///   2. 控制板上只接一个电机。
///      设置 ID 的指令对总线上的电机是“谁都听”的，若同时接了两个电机，
///      同一个 ID 会被写进两台电机，之后就无法区分左、右电机了。
///   3. 调用 SetupMotorID.begin(id) 写入 ID（1=左电机，2=右电机），例如：
///         // Motion.begin();
///         SetupMotorID.begin(SetupMotorIdClass::kLeftMotorId);
///      设置完成后恢复 Motion.begin();，并把该调用注释掉。
///
/// 设置指令（10 字节，最后一字节为 CRC8/MAXIM）：
///   AA 55 53 ID 00 00 00 00 00 CRC8
/// 正常反馈（10 字节）：
///   ID 64 00 00 00 00 00 00 00 CRC8
/// 收到并校验通过（ID 一致 + 数据体正确 + CRC 正确）后，用 esplog 打印设置成功。
///
class SetupMotorIdClass
{
public:
    // 与 MotionClass::MotorBusID 保持一致：1=左电机，2=右电机
    static constexpr uint8_t kLeftMotorId = 1;
    static constexpr uint8_t kRightMotorId = 2;

    struct Config
    {
        uint32_t rx_timeout_ms = 300; // 单次等待反馈的超时时间
        uint8_t retry = 6;            // 连续设置，第6次才能生效
    };

    // 设置电机 ID，返回 true 表示收到并校验通过正常反馈
    bool begin(uint8_t id);
    bool begin(const Config &config, uint8_t id);

private:
    bool send_set_id(uint8_t id);
    bool wait_response(uint8_t id);

    Config _config;
};

extern SetupMotorIdClass SetupMotorID;
