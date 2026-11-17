#pragma once

#include <atomic>
#include <cstdint>

#include "serial_protocol.hpp"

///
/// 自定义串口通信节点（替代 micro-ROS），功能与 master 分支保持一致：
///   下行命令：velcmd / servo_cmd / battery_current_offset / ping
///   上行状态：imu / servo_status / xl9535 / ir / motion（10Hz）、battery（1Hz）
///   保留行为：速度限幅、地检防跌落逐分量钳制、500ms 速度看门狗、链路断开停车、
///             舵机 ready 握手语义、零漂写入 NVS 并回读
///
class SerialCommClass
{
public:
    void begin(void);

private:
    static void rx_task(void *arg);
    static void status_task(void *arg);

    void send(uint8_t msg_id, const void *payload, uint8_t len);
    void handle_frame(uint8_t msg_id, const uint8_t *payload, uint8_t len);
    void apply_velcmd(float linear, float angular);

    std::atomic<int64_t> _last_velcmd_us{0};
    std::atomic<int64_t> _last_frame_us{0};
    bool _velcmd_timed_out = false;
    bool _link_connected = false;

    // 舵机状态缓存，仅 status_task 读写；单次读取失败时保留上次值
    uint8_t _servo_ready = 0;
    uint8_t _servo_id = 0;
    int32_t _servo_position = 0;
    float _servo_angle_deg = 0.0f;
    float _servo_voltage_v = 0.0f;
};

extern SerialCommClass SerialComm;
