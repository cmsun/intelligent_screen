#pragma once

#include <cstdint>

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

    /// 初始化 WiFi（micro_ros 组件 WLAN 接口）并启动 micro-ROS 通信任务。
    void begin(void);

private:
    bool _is_connect = false;
    // begin() 中初始化并配置 Agent 地址，ping 与实体创建均复用该 options
    rcl_init_options_t _init_options;
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
    static void micro_ros_task(void *arg);
};

extern MicroRosNode RosNode;
