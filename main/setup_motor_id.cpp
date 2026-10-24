#include "setup_motor_id.hpp"

#include <cstddef>
#include <span>

#include "esplog.hpp"
#include "motion.hpp" // 复用 Motion.power_on() 打开电机控制板电源

#include <fmt/ranges.h> // fmt::join

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

SetupMotorIdClass SetupMotorID;

namespace
{
// 与 motion.cpp 中的电机串口配置保持一致
constexpr uart_port_t kMotorUart = UART_NUM_1;
constexpr int kMotorUartTxPin = 18;
constexpr int kMotorUartRxPin = 17;
constexpr uint32_t kMotorUartBaud = 38400;

constexpr uint8_t kFrameSize = 10;
constexpr uint8_t kCrcPolyReflected = 0x8C; // CRC-8/MAXIM：0x31 按位反写

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
} // namespace

bool SetupMotorIdClass::begin(uint8_t id)
{
    return begin(Config{}, id);
}

bool SetupMotorIdClass::begin(const Config &config, uint8_t id)
{
    _config = config;

    // 打开电机控制板电源（与 Motion 共用 GPIO39 电源使能）
    Motion.power_on();

    // ESP-IDF UART 驱动（等效 Arduino Serial1：38400 8N1，RX=17, TX=18）
    uart_config_t uart_cfg{};
    uart_cfg.baud_rate = kMotorUartBaud;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    if (uart_driver_install(kMotorUart, 256, 0, 0, nullptr, 0) != ESP_OK)
    {
        esplog::error("set motor id {}: uart driver install failed", id);
        return false;
    }
    if (uart_param_config(kMotorUart, &uart_cfg) != ESP_OK ||
        uart_set_pin(kMotorUart, kMotorUartTxPin, kMotorUartRxPin, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK ||
        // 接收超时约 3 个字符时间（30 个波特周期），让 10 字节反馈聚成一帧
        uart_set_rx_timeout(kMotorUart, 30) != ESP_OK)
    {
        esplog::error("set motor id {}: uart config failed", id);
        uart_driver_delete(kMotorUart);
        return false;
    }

    bool ok = false;
    for (uint8_t i = 0; i < _config.retry && !ok; ++i)
    {
        uart_flush_input(kMotorUart);
        send_set_id(id);
        ok = wait_response(id);
    }

    if (!ok)
    {
        esplog::error("set motor id {} failed after {} retries", id, _config.retry);
    }

    // 释放串口，方便之后恢复 Motion.begin() 时重新初始化
    uart_driver_delete(kMotorUart);
    return ok;
}

bool SetupMotorIdClass::send_set_id(uint8_t id)
{
    // 设置指令：AA 55 53 ID 00 00 00 00 00 CRC8
    uint8_t tx[kFrameSize] = {0xAA, 0x55, 0x53, id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    tx[kFrameSize - 1] = calc_crc8(std::span(tx).first(kFrameSize - 1));

    if (uart_write_bytes(kMotorUart, tx, kFrameSize) != static_cast<int>(kFrameSize))
    {
        esplog::error("set motor id {}: uart write failed", id);
        return false;
    }
    esplog::info("set motor id {} tx: {:02X}", id, fmt::join(tx, " "));
    return true;
}

bool SetupMotorIdClass::wait_response(uint8_t id)
{
    uint8_t rx[kFrameSize] = {0};
    size_t got = 0;
    const TickType_t wait_ticks = pdMS_TO_TICKS(_config.rx_timeout_ms);

    // 反馈帧共 10 字节，可能在多次 UART_DATA 事件中到达，这里凑满一帧或超时退出
    while (got < kFrameSize)
    {
        const int n = uart_read_bytes(kMotorUart, rx + got, kFrameSize - got, wait_ticks);
        if (n <= 0)
        {
            break;
        }
        got += static_cast<size_t>(n);
    }

    if (got == 0)
    {
        esplog::error("set motor id {}: no response", id);
        return false;
    }
    if (got < kFrameSize)
    {
        esplog::error("set motor id {}: rx incomplete, {} bytes: {:02X}", id, got,
                      fmt::join(rx, rx + got, " "));
        return false;
    }

    // 正常反馈：ID 65 00 00 00 00 00 00 00 CRC8
    const bool id_ok = rx[0] == id;
    bool body_ok = (rx[1] == 0x65);
    for (size_t i = 2; i <= 8; ++i)
    {
        body_ok = (body_ok && rx[i] == 0x00);
    }
    const bool crc_ok = calc_crc8(std::span(rx).first(kFrameSize - 1)) == rx[9];

    if (!id_ok || !body_ok || !crc_ok)
    {
        esplog::error("set motor id {}: rx invalid (id {} body {} crc {}): {:02X}", id, id_ok,
                      body_ok, crc_ok, fmt::join(rx, " "));
        return false;
    }

    esplog::info("set motor id {} success, rx: {:02X}", id, fmt::join(rx, " "));
    return true;
}
