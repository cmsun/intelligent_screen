// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// 前置声明，避免与 uxr/client/transport.h 中的定义冲突
struct uxrCustomTransport;

// micro-ROS 串口传输所用的 UART 端口与波特率。
// 组件 Kconfig 只提供 TXD/RXD/RTS/CTS 引脚配置（MICROROS_UART_*），
// 端口与波特率在此处直接指定，需要与 agent 端的波特率一致。
// 引脚映射在 sdkconfig 中配置：
//   micro-ROS Settings -> UART Settings
#ifndef MICRO_ROS_SERIAL_UART_NUM
#define MICRO_ROS_SERIAL_UART_NUM (0)
#endif

#ifndef MICRO_ROS_SERIAL_BAUDRATE
#define MICRO_ROS_SERIAL_BAUDRATE (115200)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// micro-ROS 通过串口与 agent 通信的自定义 transport 回调。
// 参考 components/micro_ros/examples/int32_publisher_custom_transport 实现。

bool esp32_serial_open(struct uxrCustomTransport *transport);
bool esp32_serial_close(struct uxrCustomTransport *transport);
size_t esp32_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t esp32_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif
