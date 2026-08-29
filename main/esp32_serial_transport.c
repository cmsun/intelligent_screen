// micro-ROS 串口自定义 transport 实现（基于 ESP-IDF UART 驱动）。
// 参考 components/micro_ros/examples/int32_publisher_custom_transport。

#pragma once

#include <sdkconfig.h>

#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)

#include "esp32_serial_transport.h"

#include <uxr/client/transport.h>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "esp32_serial_transport"

#define UART_TXD (CONFIG_MICROROS_UART_TXD)
#define UART_RXD (CONFIG_MICROROS_UART_RXD)
#define UART_RTS (CONFIG_MICROROS_UART_RTS)
#define UART_CTS (CONFIG_MICROROS_UART_CTS)

#define UART_BUFFER_SIZE (512)

bool esp32_serial_open(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;

    ESP_LOGI(TAG, "Opening UART port %d (baudrate: %d)...", (int)*uart_port, (int)MICRO_ROS_SERIAL_BAUDRATE);

    uart_config_t uart_config = {
        .baud_rate = MICRO_ROS_SERIAL_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(*uart_port, &uart_config) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Failed to configure UART params on port %d", (int)*uart_port);
        return false;
    }
    if (uart_set_pin(*uart_port, UART_TXD, UART_RXD, UART_RTS, UART_CTS) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Failed to set UART pins on port %d", (int)*uart_port);
        return false;
    }
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL)
    {
        ESP_LOGE(TAG, "Failed to install UART driver on port %d", (int)*uart_port);
        return false;
    }

    ESP_LOGI(TAG, "UART port %d opened successfully", (int)*uart_port);
    return true;
}

bool esp32_serial_close(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;

    ESP_LOGI(TAG, "Closing UART port %d...", (int)*uart_port);
    bool ret = uart_driver_delete(*uart_port) == ESP_OK;
    if (ret)
    {
        ESP_LOGI(TAG, "UART port %d closed successfully", (int)*uart_port);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to close UART port %d", (int)*uart_port);
    }
    return ret;
}

size_t esp32_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    const int txBytes = uart_write_bytes(*uart_port, (const char *)buf, len);
    return txBytes;
}

size_t esp32_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    const int rxBytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    return rxBytes;
}

#endif // CONFIG_MICRO_ROS_ESP_UART_TRANSPORT
