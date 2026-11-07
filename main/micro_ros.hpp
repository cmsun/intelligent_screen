#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/rclc.h>
#include <rclc/subscription.h>

#include <chassis_msgs/msg/battery_data.h>
#include <chassis_msgs/msg/imu_data.h>
#include <chassis_msgs/msg/ir_data.h>
#include <chassis_msgs/msg/motion_data.h>
#include <chassis_msgs/msg/servo_cmd.h>
#include <chassis_msgs/msg/servo_status.h>
#include <chassis_msgs/msg/xl9535_data.h>
#include <geometry_msgs/msg/twist.h>

class MPU9250;

class MicroRosNode
{
public:
    explicit MicroRosNode(MPU9250 &imu) : _imu(imu) {}
    ~MicroRosNode() = default;

    /// 初始化传输层（WiFi 或串口，由 Kconfig 选择）并启动 micro-ROS 通信任务。
    void begin(void);

private:
    bool _is_connect = false;
    // begin() 中初始化并配置 transport 参数（UDP 地址或串口），ping 与实体创建均复用该 options
    rcl_init_options_t _init_options{};
    // 串口传输时传给 transport 回调的 UART 端口号（UDP 模式下不使用）
    size_t _serial_port = 0;
    // 零初始化：create_entities() 中途失败时 destroy_entities() 也只会对空句柄调用 fini
    rclc_executor_t _executor{};
    rclc_support_t _support{};
    rcl_allocator_t _allocator{};
    rcl_node_t _node{};
    rcl_publisher_t _imu_publisher{};
    rcl_publisher_t _servo_status_publisher{};
    rcl_publisher_t _xl9535_publisher{};
    rcl_publisher_t _ir_publisher{};
    rcl_publisher_t _motion_publisher{};
    rcl_publisher_t _battery_publisher{};
    rcl_subscription_t _velcmd_subscription{};
    rcl_subscription_t _servo_cmd_subscription{};
    chassis_msgs__msg__ImuData _imu_msg{};
    chassis_msgs__msg__ServoStatus _servo_status_msg{};
    chassis_msgs__msg__Xl9535Data _xl9535_msg{};
    chassis_msgs__msg__IrData _ir_msg{};
    chassis_msgs__msg__MotionData _motion_msg{};
    chassis_msgs__msg__BatteryData _battery_msg{};
    geometry_msgs__msg__Twist _velcmd_msg;
    chassis_msgs__msg__ServoCmd _servo_cmd_msg{};
    // 引用全局常驻的 IMU 实例（构造函数绑定）
    MPU9250 &_imu;

    // 速度指令看门狗：订阅回调写入最后收到时刻，micro_ros_task 超时后自动停车
    std::atomic<int64_t> _last_velcmd_us{0};
    bool _velcmd_timed_out = false; // 仅 micro_ros_task 访问，避免重复下发停车/刷日志

    rcl_ret_t create_entities(void);
    void destroy_entities(void);
    void sensor_data_publish(void);
    void battery_publish(void);
    void servo_status_publish(void);
    static void velcmd_subscribe_callback(const void *arg);
    static void servo_cmd_subscribe_callback(const void *arg);
    static void micro_ros_task(void *arg);
};

extern MicroRosNode RosNode;
