#include "bus_servo.hpp"

#include "esplog.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

BusServoClass BusServo;

namespace
{
// 舵机响应帧最长约 10 字节，RX 缓冲留足余量
constexpr int kUartRxBufferSize = 256;
// 0：不使用 TX 环形缓冲，uart_write_bytes 直接写入 FIFO
constexpr int kUartTxBufferSize = 0;

constexpr TickType_t kTxDoneTimeoutTicks = pdMS_TO_TICKS(100);
// 等待单个字节的超时，至少 1 个 tick，避免 CONFIG_FREERTOS_HZ 较低时退化成忙等
constexpr TickType_t kByteTimeoutTicks = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;

// Arduino millis() 语义：上电以来的毫秒数，32 位自动回绕
uint32_t millis(void) noexcept
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
} // namespace

void BusServoClass::begin(void)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_initialized.load(std::memory_order_relaxed))
    {
        return;
    }

    // 等效 Arduino Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN)
    uart_config_t uart_config{};
    uart_config.baud_rate = BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(UART_PORT, &uart_config);
    if (err == ESP_OK)
    {
        err = uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK)
    {
        err = uart_driver_install(UART_PORT, kUartRxBufferSize, kUartTxBufferSize, 0, nullptr, 0);
    }
    if (err != ESP_OK)
    {
        esplog::error("Bus servo UART{} init failed: {} ({})", static_cast<int>(UART_PORT), esp_err_to_name(err),
                      static_cast<int>(err));
        return;
    }

    _initialized.store(true, std::memory_order_relaxed);
    esplog::info("Bus servo UART{} ready: {} baud, RX=GPIO{}, TX=GPIO{}", static_cast<int>(UART_PORT),
                 static_cast<int>(BAUD_RATE), static_cast<int>(RX_PIN), static_cast<int>(TX_PIN));
}

bool BusServoClass::is_ready(void) const noexcept
{
    return _initialized.load(std::memory_order_relaxed);
}

bool BusServoClass::move_to_angle(float angle_deg, uint16_t move_time_ms)
{
    if (!std::isfinite(angle_deg) || angle_deg < MIN_ANGLE_DEG || angle_deg > MAX_ANGLE_DEG)
    {
        return false;
    }

    const uint16_t position = angle_to_position(angle_deg);
    std::array<uint8_t, 10> frame = {
        FRAME_HEADER,
        FRAME_HEADER,
        _active_id.load(std::memory_order_relaxed),
        7,
        MOVE_TIME_WRITE,
        static_cast<uint8_t>(position & 0xFFU),
        static_cast<uint8_t>(position >> 8U),
        static_cast<uint8_t>(move_time_ms & 0xFFU),
        static_cast<uint8_t>(move_time_ms >> 8U),
        0,
    };
    frame.back() = checksum(frame.data(), frame.size());

    std::lock_guard<std::mutex> lock(_mutex);
    return write_frame(frame.data(), frame.size());
}

bool BusServoClass::stop(void)
{
    std::array<uint8_t, 6> frame = {
        FRAME_HEADER, FRAME_HEADER, _active_id.load(std::memory_order_relaxed), 3, MOVE_STOP, 0,
    };
    frame.back() = checksum(frame.data(), frame.size());

    std::lock_guard<std::mutex> lock(_mutex);
    return write_frame(frame.data(), frame.size());
}

bool BusServoClass::discover_id(uint8_t &id)
{
    std::array<uint8_t, 6> request = {
        FRAME_HEADER, FRAME_HEADER, BROADCAST_ID, 3, ID_READ, 0,
    };
    request.back() = checksum(request.data(), request.size());

    std::lock_guard<std::mutex> lock(_mutex);
    // 清空 RX 中上次通信的残留，避免解析到过期数据
    uart_flush_input(UART_PORT);
    _last_rx_data.fill(0);
    _last_rx_size = 0;
    if (!write_frame(request.data(), request.size()))
    {
        _last_response_status.store(ResponseStatus::no_data, std::memory_order_relaxed);
        _last_rx_bytes.store(0, std::memory_order_relaxed);
        return false;
    }
    // 半双工总线：发完等待舵机切换回发送
    esp_rom_delay_us(TURNAROUND_DELAY_US);

    std::array<uint8_t, MAX_FRAME_SIZE> response{};
    size_t response_size = 0;
    if (!read_response(BROADCAST_ID, true, ID_READ, 7, response.data(), response_size))
    {
        return false;
    }

    const uint8_t detected_id = response[5];
    if (detected_id >= BROADCAST_ID)
    {
        _last_response_status.store(ResponseStatus::invalid_frame, std::memory_order_relaxed);
        return false;
    }
    _active_id.store(detected_id, std::memory_order_relaxed);
    id = detected_id;
    return true;
}

bool BusServoClass::read_position(uint16_t &position)
{
    uint16_t value = 0;
    if (!read_word(POSITION_READ, value) || value > MAX_POSITION)
    {
        return false;
    }
    position = value;
    return true;
}

bool BusServoClass::read_voltage_mv(uint16_t &voltage_mv)
{
    return read_word(VIN_READ, voltage_mv);
}

uint8_t BusServoClass::id(void) const
{
    return _active_id.load(std::memory_order_relaxed);
}

BusServoClass::ResponseStatus BusServoClass::last_response_status(void) const
{
    return _last_response_status.load(std::memory_order_relaxed);
}

size_t BusServoClass::last_rx_bytes(void) const
{
    return _last_rx_bytes.load(std::memory_order_relaxed);
}

size_t BusServoClass::copy_last_rx(uint8_t *buffer, size_t capacity)
{
    if (buffer == nullptr || capacity == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(_mutex);
    const size_t copy_size = std::min(capacity, _last_rx_size);
    std::copy_n(_last_rx_data.begin(), copy_size, buffer);
    return copy_size;
}

uint16_t BusServoClass::angle_to_position(float angle_deg)
{
    const float bounded_angle = std::clamp(angle_deg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    return static_cast<uint16_t>(std::lround(bounded_angle * MAX_POSITION / MAX_ANGLE_DEG));
}

float BusServoClass::position_to_angle(uint16_t position)
{
    const uint16_t bounded_position = std::clamp(position, MIN_POSITION, MAX_POSITION);
    return static_cast<float>(bounded_position) * MAX_ANGLE_DEG / MAX_POSITION;
}

uint8_t BusServoClass::checksum(const uint8_t *frame, size_t frame_size)
{
    uint16_t sum = 0;
    for (size_t index = 2; index + 1 < frame_size; ++index)
    {
        sum += frame[index];
    }
    return static_cast<uint8_t>(~sum);
}

bool BusServoClass::write_frame(const uint8_t *frame, size_t frame_size)
{
    if (!_initialized.load(std::memory_order_relaxed) || frame == nullptr || frame_size < 6)
    {
        return false;
    }
    // Arduino Serial2.write()：把整帧塞进 TX FIFO
    const int written = uart_write_bytes(UART_PORT, frame, frame_size);
    if (written != static_cast<int>(frame_size))
    {
        return false;
    }
    // Arduino Serial2.flush()：等移位寄存器发完，再让出总线给舵机应答
    uart_wait_tx_done(UART_PORT, kTxDoneTimeoutTicks);
    return true;
}

bool BusServoClass::read_word(uint8_t command, uint16_t &value)
{
    const uint8_t active_id = _active_id.load(std::memory_order_relaxed);
    std::array<uint8_t, 6> request = {
        FRAME_HEADER, FRAME_HEADER, active_id, 3, command, 0,
    };
    request.back() = checksum(request.data(), request.size());

    std::lock_guard<std::mutex> lock(_mutex);
    uart_flush_input(UART_PORT);
    _last_rx_data.fill(0);
    _last_rx_size = 0;
    if (!write_frame(request.data(), request.size()))
    {
        _last_response_status.store(ResponseStatus::no_data, std::memory_order_relaxed);
        _last_rx_bytes.store(0, std::memory_order_relaxed);
        return false;
    }
    esp_rom_delay_us(TURNAROUND_DELAY_US);

    std::array<uint8_t, MAX_FRAME_SIZE> response{};
    size_t response_size = 0;
    if (!read_response(active_id, false, command, 8, response.data(), response_size))
    {
        return false;
    }

    value = static_cast<uint16_t>(response[5]) | (static_cast<uint16_t>(response[6]) << 8U);
    return true;
}

bool BusServoClass::read_response(uint8_t expected_id,
                                  bool accept_any_id,
                                  uint8_t expected_command,
                                  size_t minimum_size,
                                  uint8_t *response,
                                  size_t &response_size)
{
    response_size = 0;
    size_t expected_size = 0;
    size_t received_bytes = 0;
    std::array<uint8_t, MAX_FRAME_SIZE> raw_data{};
    size_t raw_size = 0;
    bool invalid_data_seen = false;
    const uint32_t start_time = millis();

    while (millis() - start_time < RESPONSE_TIMEOUT_MS)
    {
        uint8_t value = 0;
        // 等效 Arduino 的 available()+read()：有字节立即返回，否则最多等一个 tick
        if (uart_read_bytes(UART_PORT, &value, 1, kByteTimeoutTicks) <= 0)
        {
            continue;
        }

        ++received_bytes;
        if (raw_size < raw_data.size())
        {
            raw_data[raw_size++] = value;
        }
        if (response_size == 0)
        {
            if (value == FRAME_HEADER)
            {
                response[response_size++] = value;
            }
            else
            {
                invalid_data_seen = true;
            }
            continue;
        }
        if (response_size == 1)
        {
            if (value == FRAME_HEADER)
            {
                response[response_size++] = value;
            }
            else
            {
                response_size = 0;
                invalid_data_seen = true;
            }
            continue;
        }

        if (response_size >= MAX_FRAME_SIZE)
        {
            response_size = 0;
            expected_size = 0;
            invalid_data_seen = true;
            continue;
        }
        response[response_size++] = value;

        if (response_size == 4)
        {
            expected_size = static_cast<size_t>(response[3]) + 3U;
            if (expected_size < 6 || expected_size > MAX_FRAME_SIZE)
            {
                response_size = 0;
                expected_size = 0;
                invalid_data_seen = true;
            }
            continue;
        }
        if (expected_size == 0 || response_size < expected_size)
        {
            continue;
        }

        const bool id_matches = accept_any_id || response[2] == expected_id;
        const bool frame_valid = response_size >= minimum_size && id_matches &&
                                 response[4] == expected_command &&
                                 response[response_size - 1] == checksum(response, response_size);
        if (frame_valid)
        {
            _last_rx_data = raw_data;
            _last_rx_size = raw_size;
            _last_response_status.store(ResponseStatus::ok, std::memory_order_relaxed);
            _last_rx_bytes.store(received_bytes, std::memory_order_relaxed);
            return true;
        }

        response_size = 0;
        expected_size = 0;
        invalid_data_seen = true;
    }

    if (response_size != 0)
    {
        invalid_data_seen = true;
    }
    _last_rx_data = raw_data;
    _last_rx_size = raw_size;
    _last_response_status.store(received_bytes == 0 && !invalid_data_seen ? ResponseStatus::no_data
                                                                         : ResponseStatus::invalid_frame,
                                std::memory_order_relaxed);
    _last_rx_bytes.store(received_bytes, std::memory_order_relaxed);
    response_size = 0;
    return false;
}
