#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/rclc.h>
#include <rclc/subscription.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

class MPU9250;

class MicroRosNode
{
public:
    explicit MicroRosNode(MPU9250 &imu) : _imu(imu) {}
    ~MicroRosNode() = default;

    /// 启动 WiFi 初始化任务与 micro-ROS 通信任务（均不阻塞调用方）。
    void begin(void);

private:
    volatile bool _wifi_connected = false; // WiFi 连接 flag，由 wifi_event_handler 在事件上下文中更新
    bool _is_connect = false;
    int64_t _last_ping_us = 0; // 上次 ping Agent 的时间戳（微秒），用于每 10 秒断连检测
    rclc_executor_t _executor;
    rclc_support_t _support;
    rcl_allocator_t _allocator;
    rcl_node_t _node;
    rcl_publisher_t _imu_publisher;
    rcl_subscription_t _velcmd_subscription;
    sensor_msgs__msg__Imu _imu_msg;
    geometry_msgs__msg__Twist _velcmd_msg;
    // 引用全局常驻的 IMU 实例（构造函数绑定）
    MPU9250 &_imu;

    rcl_ret_t create_entities(void);
    void destroy_entities(void);
    void imu_publish(void);
    static void velcmd_subscribe_callback(const void *arg);
    static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void netif_init_task(void *arg);
    static void micro_ros_task(void *arg);
};

extern MicroRosNode RosNode;
