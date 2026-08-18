#ifndef __MICRO_ROS_NODE_HPP__
#define __MICRO_ROS_NODE_HPP__

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/rclc.h>
#include <rclc/subscription.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

namespace imu {
class MPU9250;
} // namespace imu

class MicroRosNode
{
public:
    MicroRosNode() = default;
    ~MicroRosNode() = default;

    /// 初始化 WiFi（micro_ros 组件 WLAN 接口）并启动 micro-ROS 通信任务。
    /// @param imu 用于发布 IMU 数据的传感器实例
    void begin(imu::MPU9250 *imu);

private:
    bool _is_connect = false;
    rclc_executor_t _executor;
    rclc_support_t _support;
    rcl_allocator_t _allocator;
    rcl_node_t _node;
    rcl_publisher_t _imu_publisher;
    rcl_subscription_t _velcmd_subscription;
    sensor_msgs__msg__Imu _imu_msg;
    geometry_msgs__msg__Twist _velcmd_msg;
    imu::MPU9250 *_imu = nullptr;

    rcl_ret_t create_entities(void);
    void destroy_entities(void);
    void imu_publish(void);
    static void velcmd_subscribe_callback(const void *arg);
    static void micro_ros_node_task(void *arg);
};

extern MicroRosNode RosNode;

#endif
