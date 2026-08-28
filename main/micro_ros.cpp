#include <array>
#include <cstdint>

#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rcl/error_handling.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/rclc.h>
#include <rclc/subscription.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include "esplog.hpp"
#include "micro_ros.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"

// 传输方式由 micro_ros 组件自带的 Kconfig 决定：
//   micro-ROS Settings -> micro-ROS network interface select
//     - WLAN interface                -> CONFIG_MICRO_ROS_ESP_NETIF_WLAN
//     - Ethernet interface            -> CONFIG_MICRO_ROS_ESP_NETIF_ENET
//     - Micro XRCE-DDS over UART      -> CONFIG_MICRO_ROS_ESP_UART_TRANSPORT
// 据此在编译期选择不同的 transport 与网络初始化代码。
#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
extern "C"
{
#include "esp32_serial_transport.h"
}
#else
extern "C"
{
#include "uros_network_interfaces.h"
}
#endif

MicroRosNode RosNode(Imu);

#define RCCHECK(fn)                                                                                                    \
    do                                                                                                                 \
    {                                                                                                                  \
        rcl_ret_t _rc_ret = (fn);                                                                                      \
        if (_rc_ret != RCL_RET_OK)                                                                                     \
        {                                                                                                              \
            esplog::error("Failed status on line %d: %d. Aborting.", __LINE__, static_cast<int>(_rc_ret));             \
            return _rc_ret;                                                                                            \
        }                                                                                                              \
    } while (0)

#define RCLCHECK(fn)                                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        rcl_ret_t _rc_ret = (fn);                                                                                      \
        if (_rc_ret != RCL_RET_OK)                                                                                     \
        {                                                                                                              \
            esplog::error("Failed status on line %d: %d. Aborting.", __LINE__, static_cast<int>(_rc_ret));             \
            return;                                                                                                    \
        }                                                                                                              \
    } while (0)

void MicroRosNode::begin(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
    // 串口传输（custom transport）：micro-ROS 直接通过 UART 与 agent 通信，无需 WiFi
    esplog::info("micro-ROS transport: serial (UART%d, %d baud)", MICRO_ROS_SERIAL_UART_NUM,
                 MICRO_ROS_SERIAL_BAUDRATE);
#else
    // 连接 WiFi（micro_ros 组件 WLAN 网络接口，SSID/密码由 Kconfig 配置）
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    esplog::info("micro-ROS transport: UDP over WiFi");
#endif

    // 一次性配置 transport 参数，后续 ping 与实体创建均复用该 options。
    // 注意：rmw_uros_ping_agent() 无 session 时使用编译期默认地址（127.0.0.1），
    // 因此必须先把参数写入 init_options，再通过 rmw_uros_ping_agent_options() 使用。
    _init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&_init_options, rcl_get_default_allocator()) != RCL_RET_OK)
    {
        esplog::error("rcl_init_options_init failed, micro-ROS disabled");
        return;
    }
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&_init_options);

#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
    // 串口传输：注册自定义 transport 回调，args 指向 UART 端口号
    _serial_port = static_cast<size_t>(MICRO_ROS_SERIAL_UART_NUM);
    if (rmw_uros_options_set_custom_transport(true, (void *)&_serial_port, esp32_serial_open, esp32_serial_close,
                                              esp32_serial_write, esp32_serial_read, rmw_options) != RCL_RET_OK)
    {
        esplog::error("rmw_uros_options_set_custom_transport failed, micro-ROS disabled");
        return;
    }
#else
    // UDP 传输：配置 Agent 的 IP 与端口
    if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options) != RCL_RET_OK)
    {
        esplog::error("rmw_uros_options_set_udp_address failed, micro-ROS disabled");
        return;
    }
#endif

    xTaskCreatePinnedToCore(micro_ros_task, "micro_ros_node_task", 1024 * 30, this, 1, nullptr, 1);
}

rcl_ret_t MicroRosNode::create_entities(void)
{
    _allocator = rcl_get_default_allocator();

    // Agent 地址已由 begin() 配置到 _init_options，这里直接复用
    RCCHECK(rclc_support_init_with_options(&_support, 0, NULL, &_init_options, &_allocator));
    RCCHECK(rclc_node_init_default(&_node, "chassis_node", "", &_support));

    // 发布 IMU 数据
    RCCHECK(rclc_publisher_init_default(&_imu_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                        "chassis_msgs/imu"));

    // 订阅上位机速度指令（标准消息替代原 chassis_msgs/VelCmd）
    RCCHECK(rclc_subscription_init_default(&_velcmd_subscription, &_node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                           "chassis_msgs/velcmd"));

    RCCHECK(rclc_executor_init(&_executor, &_support.context, 2, &_allocator));
    RCCHECK(rclc_executor_add_subscription(&_executor, &_velcmd_subscription, &_velcmd_msg,
                                           &velcmd_subscribe_callback, ON_NEW_DATA));

    return RCL_RET_OK;
}

void MicroRosNode::destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_ret_t ret = rclc_executor_fini(&_executor);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("rclc_executor_fini failed: {}", static_cast<int>(ret));
    }

    ret = rcl_publisher_fini(&_imu_publisher, &_node);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("rcl_publisher_fini failed: {}", static_cast<int>(ret));
    }

    ret = rcl_subscription_fini(&_velcmd_subscription, &_node);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("rcl_subscription_fini failed: {}", static_cast<int>(ret));
    }

    ret = rcl_node_fini(&_node);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("rcl_node_fini failed: {}", static_cast<int>(ret));
    }

    ret = rclc_support_fini(&_support);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("rclc_support_fini failed: {}", static_cast<int>(ret));
    }
}

void MicroRosNode::imu_publish(void)
{
    const ImuSample &s = _imu.last_sample();
    const int64_t now_us = esp_timer_get_time();

    _imu_msg.header.stamp.sec = static_cast<int32_t>(now_us / 1000000);
    _imu_msg.header.stamp.nanosec = static_cast<uint32_t>((now_us % 1000000) * 1000);
    rosidl_runtime_c__String__assign(&_imu_msg.header.frame_id, "imu_link");

    const std::array<float, 4> q = _imu.quaternion(); // (w, x, y, z)
    _imu_msg.orientation.x = q[1];
    _imu_msg.orientation.y = q[2];
    _imu_msg.orientation.z = q[3];
    _imu_msg.orientation.w = q[0];

    _imu_msg.angular_velocity.x = s.gyro.x;
    _imu_msg.angular_velocity.y = s.gyro.y;
    _imu_msg.angular_velocity.z = s.gyro.z;

    _imu_msg.linear_acceleration.x = s.accel.x;
    _imu_msg.linear_acceleration.y = s.accel.y;
    _imu_msg.linear_acceleration.z = s.accel.z;

    // 协方差未知
    _imu_msg.orientation_covariance[0] = -1.0;
    _imu_msg.angular_velocity_covariance[0] = -1.0;
    _imu_msg.linear_acceleration_covariance[0] = -1.0;

    const rcl_ret_t ret = rcl_publish(&_imu_publisher, &_imu_msg, nullptr);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("rcl_publish failed: {}", static_cast<int>(ret));
    }
}

void MicroRosNode::velcmd_subscribe_callback(const void *arg)
{
    const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(arg);
    Motion.set_chassis_speed(static_cast<float>(msg->linear.x), static_cast<float>(msg->angular.z));
}

void MicroRosNode::micro_ros_task(void *arg)
{
    esp_task_wdt_add(nullptr);
    auto *self = static_cast<MicroRosNode *>(arg);
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&self->_init_options);
    int64_t last_ping_us = esp_timer_get_time();

    while (true)
    {
        // 无论是否已连接，每轮循环都喂狗，避免等待 Agent 上线期间看门狗超时
        esp_task_wdt_reset();

        if (self->_is_connect)
        {
            self->imu_publish();

            if (rclc_executor_spin_some(&self->_executor, RCL_MS_TO_NS(10)) == RCL_RET_OK)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // 每 5 秒 ping 一次 agent，检测连接是否断开；断开则销毁实体，回到重连流程
            const int64_t now_us = esp_timer_get_time();
            if (now_us - last_ping_us >= 5LL * 1000 * 1000)
            {
                last_ping_us = now_us;
                if (rmw_uros_ping_agent_options(100, 3, rmw_options) != RCL_RET_OK)
                {
                    esplog::warn("micro-ROS agent lost, reconnecting ...");
                    self->destroy_entities();
                    self->_is_connect = false;
                }
            }
        }
        else
        {
            if (rmw_uros_ping_agent_options(100, 3, rmw_options) == RCL_RET_OK)
            {
                if(self->create_entities() == RCL_RET_OK)
                {
                    self->_is_connect = true;
                    last_ping_us = esp_timer_get_time(); // 重置连接检测计时基准，避免刚连上就触发第一次 ping
                }
                else
                {
                    esplog::error("micro-ROS agent connected, but create entities failed");
                }
            }
            else
            {
                esplog::warn("micro-ROS agent not reachable, retrying ...");
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
