#include <algorithm>
#include <array>

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

#include "bus_servo.hpp"
#include "esplog.hpp"
#include "micro_ros.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"

namespace
{
// ---- 与上位机（OrangePi 手柄节点）的指令约定 ----
// chassis_msgs/velcmd         geometry_msgs/Twist  linear.x [m/s]、angular.z [rad/s]
// chassis_msgs/servo_cmd      std_msgs/Int32       data = 目标角度 deg
//                             通信层不做角度限制，限位统一在 BusServoClass 驱动层
// chassis_msgs/servo_status   chassis_msgs/ServoStatus  ESP32 周期上报舵机状态
// 上位机需持续发布速度指令（建议 >= 20Hz）；超过 kVelcmdTimeoutUs 没收到即自动停车。
constexpr float kMaxLinearMps = 0.6f;  // 约 36 m/min，接近减速比后的电机上限
constexpr float kMaxAngularRps = 2.0f; // 约 115 deg/s
constexpr int64_t kVelcmdTimeoutUs = 500LL * 1000;
// 串口 115200 下 100Hz 的 IMU 帧会挤占速度指令带宽，降到 10Hz
constexpr int64_t kImuPublishPeriodUs = 100LL * 1000;

// 舵机状态：位置 10Hz（与上位机手柄 10Hz 重算目标同步）、电压 0.5Hz
// 半双工读会短暂阻塞，10Hz 下会占用部分速度指令处理时间
constexpr int64_t kServoStatusPeriodUs = 100LL * 1000;
constexpr int64_t kServoVoltagePeriodUs = 2000LL * 1000;
// 舵机命令：micro-ROS 层不做任何角度限制，只按目标角度去重
constexpr uint16_t kServoMoveTimeMs = 300;
} // namespace

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
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN)
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

    // 发布舵机状态（自定义消息）；上位机收到 ready=true 后才下发舵机命令
    RCCHECK(rclc_publisher_init_default(&_servo_status_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, ServoStatus),
                                        "chassis_msgs/servo_status"));

    // 订阅上位机速度指令
    RCCHECK(rclc_subscription_init_default(&_velcmd_subscription, &_node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                           "chassis_msgs/velcmd"));

    // 订阅舵机目标角度（标准 Int32，deg；角度限位由 BusServoClass 驱动层负责）
    RCCHECK(rclc_subscription_init_default(&_servo_cmd_subscription, &_node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                           "chassis_msgs/servo_cmd"));

    // 2 个订阅 + 余量
    RCCHECK(rclc_executor_init(&_executor, &_support.context, 4, &_allocator));
    RCCHECK(rclc_executor_add_subscription(&_executor, &_velcmd_subscription, &_velcmd_msg,
                                           &velcmd_subscribe_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&_executor, &_servo_cmd_subscription, &_servo_cmd_msg,
                                           &servo_cmd_subscribe_callback, ON_NEW_DATA));

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

    ret = rcl_publisher_fini(&_servo_status_publisher, &_node);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("servo status publisher fini failed: {}", static_cast<int>(ret));
    }

    ret = rcl_subscription_fini(&_velcmd_subscription, &_node);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("rcl_subscription_fini failed: {}", static_cast<int>(ret));
    }

    ret = rcl_subscription_fini(&_servo_cmd_subscription, &_node);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("servo cmd subscription fini failed: {}", static_cast<int>(ret));
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

void MicroRosNode::servo_status_publish(void)
{
    const rcl_ret_t ret = rcl_publish(&_servo_status_publisher, &_servo_status_msg, nullptr);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("servo status publish failed: {}", static_cast<int>(ret));
    }
}

void MicroRosNode::velcmd_subscribe_callback(const void *arg)
{
    const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(arg);

    // MotionClass::set_chassis_speed() 与 Twist 同为 SI 单位（m/s、rad/s），
    // 单位换算在 MotionClass 内部完成，这里只做限幅后直接下发。
    const float linear = std::clamp(static_cast<float>(msg->linear.x), -kMaxLinearMps, kMaxLinearMps);
    const float angular = std::clamp(static_cast<float>(msg->angular.z), -kMaxAngularRps, kMaxAngularRps);
    Motion.set_chassis_speed(linear, angular);

    // 刷新看门狗时间戳，供 micro_ros_task 判断上位机是否还在持续下发
    RosNode._last_velcmd_us.store(esp_timer_get_time(), std::memory_order_release);
}

void MicroRosNode::servo_cmd_subscribe_callback(const void *arg)
{
    const auto *msg = static_cast<const std_msgs__msg__Int32 *>(arg);
    const int32_t angle_deg = msg->data;

    // 总线未就绪时静默忽略
    if (!BusServo.is_ready())
    {
        return;
    }

    // Int32 已是角度（deg），直接透传；限位由 BusServo.move_to_angle() 完成
    if (!BusServo.move_to_angle(static_cast<float>(angle_deg), kServoMoveTimeMs))
    {
        esplog::warn("bus servo move to {} deg failed", angle_deg);
    }
}

void MicroRosNode::micro_ros_task(void *arg)
{
    esp_task_wdt_add(nullptr);
    int64_t last_ping_us = esp_timer_get_time();
    int64_t last_imu_publish_us = 0;
    int64_t last_servo_status_us = 0;
    int64_t last_servo_voltage_us = 0;
    auto *self = static_cast<MicroRosNode *>(arg);
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&self->_init_options);

    while (true)
    {
        // 无论是否已连接，每轮循环都喂狗，避免等待 Agent 上线期间看门狗超时
        esp_task_wdt_reset();

        if (self->_is_connect)
        {
            const int64_t now_us = esp_timer_get_time();

            // 串口 115200 下按 100Hz 发布 IMU 会挤占速度指令带宽，这里降到 10Hz
            if (now_us - last_imu_publish_us >= kImuPublishPeriodUs)
            {
                last_imu_publish_us = now_us;
                self->imu_publish();
            }

            // 只要成功读到过一次位置即视为就绪（握手语义），ready 保持为 true；
            // 单次读取失败时保留上次的 position/angle_deg，避免上位机因 ready=false
            // 反复跳过目标角度计算导致完全无法控制。
            if (now_us - last_servo_status_us >= kServoStatusPeriodUs)
            {
                last_servo_status_us = now_us;
                uint16_t position = 0;
                if (BusServo.is_ready() && BusServo.read_position(position))
                {
                    self->_servo_status_msg.ready = true;
                    self->_servo_status_msg.id = BusServo.id();
                    self->_servo_status_msg.position = static_cast<int32_t>(position);
                    self->_servo_status_msg.angle_deg = BusServoClass::position_to_angle(position);
                }

                if (now_us - last_servo_voltage_us >= kServoVoltagePeriodUs)
                {
                    last_servo_voltage_us = now_us;
                    uint16_t voltage_mv = 0;
                    if (BusServo.is_ready() && BusServo.read_voltage_mv(voltage_mv))
                    {
                        self->_servo_status_msg.voltage_v = static_cast<float>(voltage_mv) / 1000.0f;
                    }
                }
                self->servo_status_publish();
            }

            if (rclc_executor_spin_some(&self->_executor, RCL_MS_TO_NS(10)) == RCL_RET_OK)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // 手柄断连 / 上位机卡住时，不能让底盘继续跑最后一条速度指令
            const int64_t last_cmd_us = self->_last_velcmd_us.load(std::memory_order_acquire);
            if (last_cmd_us != 0 && now_us - last_cmd_us > kVelcmdTimeoutUs)
            {
                if (!self->_velcmd_timed_out)
                {
                    Motion.set_chassis_speed(0.0f, 0.0f);
                    self->_velcmd_timed_out = true;
                    esplog::warn("velcmd timeout, chassis stopped");
                }
            }
            else
            {
                self->_velcmd_timed_out = false;
            }

            // 每 5 秒 ping 一次 agent，检测连接是否断开；断开则销毁实体，回到重连流程
            if (now_us - last_ping_us >= 5LL * 1000 * 1000)
            {
                last_ping_us = now_us;
                // 已连接：必须走 session 版 ping，复用现有串口，不再 open/close
                if (rmw_uros_ping_agent(100, 3) != RCL_RET_OK)
                {
                    esplog::warn("micro-ROS agent lost, reconnecting ...");
                    self->_is_connect = false;
                    self->destroy_entities();
                    Motion.set_chassis_speed(0.0f, 0.0f); // 断链立即停车
                    self->_last_velcmd_us.store(0, std::memory_order_release);
                }
            }
        }
        else
        {
            // 未连接：ping 成功才创建实体
            // 走 options 版 ping，传入初始化选项，如果是串口传输，则会自动 open/close 串口
            if (rmw_uros_ping_agent_options(100, 3, rmw_options) == RCL_RET_OK)
            {
                if(self->create_entities() == RCL_RET_OK)
                {
                    self->_is_connect = true;
                    self->_last_velcmd_us.store(0, std::memory_order_release);
                    self->_velcmd_timed_out = false;
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
