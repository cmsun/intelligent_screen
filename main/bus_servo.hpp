#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include <driver/gpio.h>
#include <driver/uart.h>

/// 半双工串口总线舵机驱动（原 Arduino 版 bus_servo 的 ESP-IDF 移植）。
///
/// 接线（ESP32-S3 侧，与原 Arduino Serial2 一致）：
///   GPIO11 -> 舵机总线 RX
///   GPIO12 -> 舵机总线 TX
///
/// 串口分配：
///   UART0：console / micro-ROS 串口传输
///   UART1：底盘电机控制板
///   UART2：总线舵机
///
/// 帧格式：0x55 0x55 ID LEN CMD [DATA...] CHECKSUM
///   LEN  = 数据字节数 + 3（CMD 计入），整帧长度 = LEN + 3
///   校验和 = ~(ID + LEN + CMD + DATA...) 取低 8 位
class BusServoClass
{
public:
    enum class ResponseStatus : uint8_t
    {
        ok,
        no_data,
        invalid_frame,
    };

    static constexpr uart_port_t UART_PORT = UART_NUM_2;
    static constexpr uint32_t BAUD_RATE = 115200;
    static constexpr gpio_num_t RX_PIN = GPIO_NUM_11;
    static constexpr gpio_num_t TX_PIN = GPIO_NUM_12;
    static constexpr uint8_t DEFAULT_ID = 1;
    static constexpr uint16_t DEFAULT_MOVE_TIME_MS = 1000;
    static constexpr uint16_t MIN_POSITION = 0;
    static constexpr uint16_t MAX_POSITION = 1000;
    static constexpr float MIN_ANGLE_DEG = 0.0f;
    static constexpr float MAX_ANGLE_DEG = 240.0f;

    /// 初始化舵机总线 UART（等效 Arduino 的 Serial2.begin）
    void begin(void);
    /// UART 初始化成功，可以收发
    [[nodiscard]] bool is_ready(void) const noexcept;
    /// 以指定时间转到目标角度（超时未到位由舵机自行停止）
    bool move_to_angle(float angle_deg, uint16_t move_time_ms = DEFAULT_MOVE_TIME_MS);
    /// 停止当前转动
    bool stop(void);
    /// 广播查询总线上舵机 ID，成功后更新并保存为当前 ID
    bool discover_id(uint8_t &id);
    /// 读取当前位置（0~1000）
    bool read_position(uint16_t &position);
    /// 读取舵机供电电压 [mV]
    bool read_voltage_mv(uint16_t &voltage_mv);
    uint8_t id(void) const;
    ResponseStatus last_response_status(void) const;
    size_t last_rx_bytes(void) const;
    size_t copy_last_rx(uint8_t *buffer, size_t capacity);
    static uint16_t angle_to_position(float angle_deg);
    static float position_to_angle(uint16_t position);

private:
    static constexpr uint8_t FRAME_HEADER = 0x55;
    static constexpr uint8_t MOVE_TIME_WRITE = 1;
    static constexpr uint8_t MOVE_STOP = 12;
    static constexpr uint8_t ID_READ = 14;
    static constexpr uint8_t VIN_READ = 27;
    static constexpr uint8_t POSITION_READ = 28;
    static constexpr uint8_t BROADCAST_ID = 0xFE;
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 20;
    static constexpr uint32_t TURNAROUND_DELAY_US = 600;
    static constexpr size_t MAX_FRAME_SIZE = 32;

    std::atomic<bool> _initialized{false};
    std::mutex _mutex;
    std::array<uint8_t, MAX_FRAME_SIZE> _last_rx_data{};
    size_t _last_rx_size = 0;
    std::atomic<uint8_t> _active_id{DEFAULT_ID};
    std::atomic<ResponseStatus> _last_response_status{ResponseStatus::no_data};
    std::atomic<size_t> _last_rx_bytes{0};

    static uint8_t checksum(const uint8_t *frame, size_t frame_size);
    bool write_frame(const uint8_t *frame, size_t frame_size);
    bool read_word(uint8_t command, uint16_t &value);
    bool read_response(uint8_t expected_id,
                       bool accept_any_id,
                       uint8_t expected_command,
                       size_t minimum_size,
                       uint8_t *response,
                       size_t &response_size);
};

extern BusServoClass BusServo;
