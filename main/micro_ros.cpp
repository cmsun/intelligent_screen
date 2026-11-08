#include <algorithm>

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

#include "battery.hpp"
#include "bus_servo.hpp"
#include "esplog.hpp"
#include "ir_receiver.hpp"
#include "micro_ros.hpp"
#include "motion.hpp"
#include "mpu9250.hpp"
#include "xl9535.hpp"

namespace
{
// ---- 与上位机（OrangePi 手柄节点）的指令约定 ----
// 订阅：
//   chassis_msgs/velcmd         geometry_msgs/Twist     linear.x [m/s]、angular.z [rad/s]
//   chassis_msgs/servo_cmd      chassis_msgs/ServoCmd  angle_deg [deg]、move_time_ms [ms]
// 发布（均由 ESP32 周期上报）：
//   chassis_msgs/imu            chassis_msgs/ImuData       欧拉角 + yaw 角速度
//   chassis_msgs/servo_status   chassis_msgs/ServoStatus   舵机位置/电压
//   chassis_msgs/xl9535         chassis_msgs/Xl9535Data    地面检测/障碍物检测/风扇/中断
//   chassis_msgs/ir             chassis_msgs/IrData        红外帧与左右通道活动
//   chassis_msgs/motion         chassis_msgs/MotionData    底盘速度/电机状态
//   chassis_msgs/battery        chassis_msgs/BatteryData   电池电压/电流/SOC
// 上位机需持续发布速度指令（建议 >= 20Hz）；超过 kVelcmdTimeoutUs 没收到即自动停车。
constexpr float kMaxLinearMps = 0.6f;  // 约 36 m/min，接近减速比后的电机上限
constexpr float kMaxAngularRps = 2.0f; // 约 115 deg/s
constexpr int64_t kVelcmdTimeoutUs = 500LL * 1000;
// 串口 115200 下除电池外的状态统一 10Hz 上报，避免挤占速度指令带宽
constexpr int64_t kStatusPublishPeriodUs = 100LL * 1000;
// 电池电压/电流/SOC 变化慢，单独按 1Hz 上报
constexpr int64_t kBatteryPublishPeriodUs = 1000LL * 1000;
// 舵机电压 0.5Hz；半双工读位置会短暂阻塞，10Hz 下会占用部分速度指令处理时间
constexpr int64_t kServoVoltagePeriodUs = 2000LL * 1000;

// micro-ROS 自定义消息都是 POD 结构，统一在此发布并告警
template <typename Msg>
void publish_msg(const rcl_publisher_t &publisher, const Msg &msg, const char *name)
{
    const rcl_ret_t ret = rcl_publish(&publisher, &msg, nullptr);
    if (ret != RCL_RET_OK)
    {
        esplog::warn("{} publish failed: {}", name, static_cast<int>(ret));
    }
}
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

MicroRosNode RosNode;

#define RCCHECK(fn)                                                                                                    \
    do                                                                                                                 \
    {                                                                                                                  \
        rcl_ret_t _rc_ret = (fn);                                                                                      \
        if (_rc_ret != RCL_RET_OK)                                                                                     \
        {                                                                                                              \
            esplog::error("Failed status on line {}: {}. Aborting.", __LINE__, static_cast<int>(_rc_ret));             \
            return _rc_ret;                                                                                            \
        }                                                                                                              \
    } while (0)

#define RCLCHECK(fn)                                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        rcl_ret_t _rc_ret = (fn);                                                                                      \
        if (_rc_ret != RCL_RET_OK)                                                                                     \
        {                                                                                                              \
            esplog::error("Failed status on line {}: {}. Aborting.", __LINE__, static_cast<int>(_rc_ret));             \
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

    // 发布 IMU 姿态（自定义精简消息：欧拉角 + yaw 角速度）
    RCCHECK(rclc_publisher_init_default(&_imu_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, ImuData),
                                        "chassis_msgs/imu"));

    // 发布舵机状态（自定义消息）；上位机收到 ready=true 后才下发舵机命令
    RCCHECK(rclc_publisher_init_default(&_servo_status_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, ServoStatus),
                                        "chassis_msgs/servo_status"));

    RCCHECK(rclc_publisher_init_default(&_xl9535_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, Xl9535Data),
                                        "chassis_msgs/xl9535"));
    RCCHECK(rclc_publisher_init_default(&_ir_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, IrData),
                                        "chassis_msgs/ir"));
    RCCHECK(rclc_publisher_init_default(&_motion_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, MotionData),
                                        "chassis_msgs/motion"));
    RCCHECK(rclc_publisher_init_default(&_battery_publisher, &_node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, BatteryData),
                                        "chassis_msgs/battery"));

    // 订阅上位机速度指令
    RCCHECK(rclc_subscription_init_default(&_velcmd_subscription, &_node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                           "chassis_msgs/velcmd"));

    // 订阅舵机命令（目标角度 deg + 转动时间 ms；角度限位由 BusServoClass 驱动层负责）
    RCCHECK(rclc_subscription_init_default(&_servo_cmd_subscription, &_node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, ServoCmd),
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
    // support 尚未初始化时（create_entities 在 rclc_support_init 就失败）没有实体可清理
    if (!rcl_context_is_valid(&_support.context))
    {
        return;
    }
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

    rcl_publisher_t *publishers[] = {&_xl9535_publisher, &_ir_publisher,
                                     &_motion_publisher, &_battery_publisher};
    for (rcl_publisher_t *publisher : publishers)
    {
        ret = rcl_publisher_fini(publisher, &_node);
        if (ret != RCL_RET_OK)
        {
            esplog::warn("sensor publisher fini failed: {}", static_cast<int>(ret));
        }
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

void MicroRosNode::sensor_data_publish(void)
{
    // IMU：只上报欧拉角与 yaw 角速度（deg、deg/s）
    if (Imu.is_ready())
    {
        const Vec3 euler = Imu.euler_deg();
        _imu_msg.roll_deg = euler.x;
        _imu_msg.pitch_deg = euler.y;
        _imu_msg.yaw_deg = euler.z;
        _imu_msg.yaw_rate_deg_s = Imu.yaw_rate_deg_s();
    }
    publish_msg(_imu_publisher, _imu_msg, "imu");

    // XL9535：四路地面检测 + 头顶/下巴障碍物检测 + 风扇/中断状态
    _xl9535_msg.floor_left = Xl9535.floor_detected(0);
    _xl9535_msg.floor_left_middle = Xl9535.floor_detected(1);
    _xl9535_msg.floor_right_middle = Xl9535.floor_detected(2);
    _xl9535_msg.floor_right = Xl9535.floor_detected(3);
    _xl9535_msg.head_up_obstacle = Xl9535.head_up_obstacle();
    _xl9535_msg.head_down_obstacle = Xl9535.head_down_obstacle();
    _xl9535_msg.fan_on = Xl9535.fan_on();
    _xl9535_msg.int_active = Xl9535.int_active();
    publish_msg(_xl9535_publisher, _xl9535_msg, "xl9535");

    // 红外：最近一帧 NEC 解码结果 + 左右通道 500ms 内活动
    const IrFrame frame = IrReceiver.last_frame();
    _ir_msg.side = static_cast<uint8_t>(frame.side == IrSide::Right ? 1 : 0);
    _ir_msg.address = frame.address;
    _ir_msg.command = frame.command;
    _ir_msg.raw_code = frame.raw_code;
    _ir_msg.address_valid = frame.address_valid;
    _ir_msg.repeat = frame.repeat;
    _ir_msg.timestamp_ms = frame.timestamp_ms;
    _ir_msg.left_active = IrReceiver.signal_seen(IrSide::Left, 500);
    _ir_msg.right_active = IrReceiver.signal_seen(IrSide::Right, 500);
    publish_msg(_ir_publisher, _ir_msg, "ir");

    // Motion：目标/当前速度、左右电机 rpm 与状态（速度已换算为 SI）
    const MotionClass::Status motion = Motion.status();
    _motion_msg.linear_speed_cur = motion.linear_speed_cur;
    _motion_msg.angular_speed_cur = motion.angular_speed_cur;
    _motion_msg.left_motor_rpm = motion.left_motor_rpm;
    _motion_msg.right_motor_rpm = motion.right_motor_rpm;
    _motion_msg.left_motor_state = motion.left_motor_state;
    _motion_msg.right_motor_state = motion.right_motor_state;
    _motion_msg.left_motor_mode = motion.left_motor_mode;
    _motion_msg.right_motor_mode = motion.right_motor_mode;
    _motion_msg.left_communication_error = motion.left_communication_error;
    _motion_msg.right_communication_error = motion.right_communication_error;
    publish_msg(_motion_publisher, _motion_msg, "motion");

}

void MicroRosNode::battery_publish(void)
{
    // Battery：电压/电流/功率/SOC/状态；按 1Hz 独立上报
    const BatteryClass::Status battery = Battery.status();
    _battery_msg.battery_voltage = battery.battery_voltage;
    _battery_msg.charger_voltage = battery.charger_voltage;
    _battery_msg.current = battery.current;
    _battery_msg.power = battery.power;
    _battery_msg.soc = battery.soc;
    _battery_msg.state = static_cast<uint8_t>(battery.state);
    _battery_msg.charge_enabled = battery.charge_enabled;
    publish_msg(_battery_publisher, _battery_msg, "battery");
}

void MicroRosNode::servo_status_publish(void)
{
    publish_msg(_servo_status_publisher, _servo_status_msg, "servo status");
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
    const auto *msg = static_cast<const chassis_msgs__msg__ServoCmd *>(arg);
    const float angle_deg = msg->angle_deg;
    const uint16_t move_time_ms = msg->move_time_ms;

    // 总线未就绪时静默忽略
    if (!BusServo.is_ready())
    {
        return;
    }

    // 目标角度与转动时间均来自上位机；限位由 BusServo.move_to_angle() 完成
    if (!BusServo.move_to_angle(angle_deg, move_time_ms))
    {
        esplog::warn("bus servo move to {} deg in {} ms failed", angle_deg, move_time_ms);
    }
}

void MicroRosNode::micro_ros_task(void *arg)
{
    esp_task_wdt_add(nullptr);
    int64_t last_ping_us = esp_timer_get_time();
    int64_t last_status_publish_us = 0;
    int64_t last_battery_publish_us = 0;
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

            // 串口 115200 下除电池外的状态统一 10Hz 上报，避免挤占速度指令带宽
            if (now_us - last_status_publish_us >= kStatusPublishPeriodUs)
            {
                last_status_publish_us = now_us;
                self->sensor_data_publish();

                // 只要成功读到过一次位置即视为就绪（握手语义），ready 保持为 true；
                // 单次读取失败时保留上次的 position/angle_deg，避免上位机因 ready=false
                // 反复跳过目标角度计算导致完全无法控制。
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

            // 电池量变化慢，按 1Hz 单独上报，不随 10Hz 状态帧一起发
            if (now_us - last_battery_publish_us >= kBatteryPublishPeriodUs)
            {
                last_battery_publish_us = now_us;
                self->battery_publish();
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
                    // 清理已创建的实体，否则静态实体池被占满，后续重试只会继续失败
                    self->destroy_entities();
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
