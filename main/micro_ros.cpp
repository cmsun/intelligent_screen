#include <array>
#include <cstdint>

#include <esp_event.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <esp_wifi.h>
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

extern "C"
{
#include "uros_network_interfaces.h"
}

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
    // WiFi 初始化放到独立任务：uros_network_interface_initialize() 内部会阻塞等待
    // 连上或耗尽重试次数（portMAX_DELAY），直接在这里调用会卡死 begin()。
    // 连上 WiFi（拿到 IP）后 _wifi_connected 被置位，micro_ros_task 据此再创建 ROS 实体。
    xTaskCreatePinnedToCore(netif_init_task, "netif_init", 1024 * 6, this, 1, nullptr, 1);
    xTaskCreatePinnedToCore(micro_ros_task, "micro_ros_task", 1024 * 30, this, 1, nullptr, 1);
}

void MicroRosNode::wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    auto *self = static_cast<MicroRosNode *>(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        const auto *reason = static_cast<wifi_event_sta_disconnected_t *>(event_data);
        self->_wifi_connected = false;
        esplog::warn("WiFi disconnected, reason: {}", static_cast<int>(reason->reason));
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        self->_wifi_connected = true;
        esplog::info("WiFi connected, got IP");
    }
}

void MicroRosNode::netif_init_task(void *arg)
{
    auto *self = static_cast<MicroRosNode *>(arg);
    esp_task_wdt_add(nullptr);

    // 注册事件处理器：WiFi 断开 / 拿到 IP 时由 wifi_event_handler 更新 _wifi_connected
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_event_handler, self));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, self));

    // 阻塞式初始化：最多重试 CONFIG_ESP_MAXIMUM_RETRY 次，失败后 WiFi 停在断开状态。
    // 因为运行在独立任务中，阻塞不影响其他任务。
    uros_network_interface_initialize();

    // 判断 _wifi_connected：未连接则不断重连；已连接则等待断开事件清除 flag 后自动进入重连循环
    while (true)
    {
        esp_task_wdt_reset();

        if (!self->_wifi_connected)
        {
            // SSID/密码组件已配置并 esp_wifi_start()，这里只需反复调用 esp_wifi_connect()，
            // 直到 GOT_IP 事件将 _wifi_connected 置位
            esplog::warn("WiFi not connected, reconnecting ...");
            esp_wifi_connect();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        else
        {
            // 已连接：_wifi_connected 由断开事件清除，此处只需轮询检测状态变化
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

rcl_ret_t MicroRosNode::create_entities(void)
{
    _allocator = rcl_get_default_allocator();

    // 先零初始化所有实体对象，保证中途失败后 destroy_entities() 可以安全清理半成品
    _support = {};
    _node = rcl_get_zero_initialized_node();
    _imu_publisher = rcl_get_zero_initialized_publisher();
    _velcmd_subscription = rcl_get_zero_initialized_subscription();
    _executor = rclc_executor_get_zero_initialized_executor();

    // 配置 Agent 的 IP 与端口（UDP 传输）
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, _allocator));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    RCCHECK(rclc_support_init_with_options(&_support, 0, NULL, &init_options, &_allocator));
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
    auto *self = static_cast<MicroRosNode *>(arg);
    esp_task_wdt_add(nullptr);

    while (true)
    {
        // 无论是否已连接，每轮循环都喂狗，避免等待 Agent 上线期间看门狗超时
        esp_task_wdt_reset();

        // 等待 WiFi 就绪（_wifi_connected 由 wifi_event_handler 维护），未就绪则轮询等待
        if (!self->_wifi_connected)
        {
            if(self->_is_connect)
            {
                self->destroy_entities();
                self->_is_connect = false;
            }
            esplog::info("Waiting for WiFi ...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (self->_is_connect)
        {
            self->imu_publish();
            rclc_executor_spin_some(&self->_executor, RCL_MS_TO_NS(100));

            // 每 10 秒 ping 一次 Agent 检测断连；断连则销毁实体并重置连接状态
            const int64_t now_us = esp_timer_get_time();
            if (now_us - self->_last_ping_us >= 10 * 1000000)
            {
                self->_last_ping_us = now_us;
                if (rmw_uros_ping_agent(100, 3) != RCL_RET_OK)
                {
                    esplog::warn("Agent lost, destroying entities ...");
                    self->destroy_entities();
                    self->_is_connect = false;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            // 等待 Agent 上线后建立实体
            if (rmw_uros_ping_agent(100, 3) == RCL_RET_OK)
            {
                if (self->create_entities() == RCL_RET_OK)
                {
                    self->_is_connect = true;
                    esplog::info("micro-ROS connected, publishing chassis_msgs/imu ...");
                }
                else
                {
                    // 创建失败：清理半成品实体，下一轮重新尝试
                    self->destroy_entities();
                    esplog::error("create_entities failed, clean up and retry later");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
