#pragma once

#include <atomic>
#include <cstdint>
#include <ostream>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

// fmtlib 自定义类型的字符串格式化需包含 <fmt/ostream.h>
#include <fmt/ostream.h>

///
/// XL9535 16 位 I2C GPIO 扩展芯片驱动（与 PCA9535 寄存器兼容）
///
/// 接线（ESP32-S3 侧）：
///   GPIO35 -> XL9535 SCL
///   GPIO36 -> XL9535 SDA
///   GPIO34 -> XL9535 INT（中断输出）
///
/// 寄存器（PCA9535 兼容，支持地址自增）：
///   0x00 输入端口 0（IO0_0~IO0_7）
///   0x01 输入端口 1（IO1_0~IO1_7）
///   0x02 输出端口 0
///   0x03 输出端口 1
///   0x04 极性反转 0
///   0x05 极性反转 1
///   0x06 方向配置 0（1=输入，0=输出）
///   0x07 方向配置 1
///
class Xl9535Class
{
public:
    struct Config
    {
        gpio_num_t scl_pin = GPIO_NUM_35;
        gpio_num_t sda_pin = GPIO_NUM_36;
        gpio_num_t int_pin = GPIO_NUM_34;
        uint8_t device_address = 0x20;
        uint32_t i2c_freq_hz = 100000;
        // 单次 I2C 传输超时（ms）；-1 表示永久等待，总线异常时会一直卡住
        int i2c_timeout_ms = 100;
        uint32_t poll_period_ms = 100;
    };

    // 管脚编号：io0_x 对应端口 0，io1_x 对应端口 1
    enum class Pin : uint8_t
    {
        io0_0 = 0,
        io0_1,
        io0_2,
        io0_3,
        io0_4,
        io0_5,
        io0_6,
        io0_7,
        io1_0,
        io1_1,
        io1_2,
        io1_3,
        io1_4,
        io1_5,
        io1_6,
        io1_7,
    };

    // 本项目的功能管脚定义
    static constexpr Pin fan_pin = Pin::io1_1;                // 风扇控制，高电平转
    static constexpr Pin floor_left_pin = Pin::io1_2;         // 左地面检测
    static constexpr Pin floor_left_middle_pin = Pin::io1_3;  // 左中地面检测
    static constexpr Pin floor_right_middle_pin = Pin::io1_4; // 右中地面检测
    static constexpr Pin floor_right_pin = Pin::io0_4;        // 右地面检测
    static constexpr Pin head_up_obstacle_pin = Pin::io0_6;   // 头顶障碍物检测，低有效
    static constexpr Pin head_down_obstacle_pin = Pin::io0_7; // 下巴障碍物检测，低有效

    bool begin(void);
    bool begin(const Config &config);
    // 刷新输入端口缓存（一次读取两个端口）
    bool refresh(void);
    // 输出单个管脚电平
    bool set_pin(Pin pin, bool level);
    // 读取单个管脚原始电平（来自缓存，需先 refresh）
    bool read_pin(Pin pin, bool &level) const;
    // 风扇控制
    bool set_fan(bool on);
    bool fan_on(void) const;
    // 地面检测：index 0=左 1=左中 2=右中 3=右；信号低电平有效，
    // 读到 0 = detected（检测到地面），读到 1 = lost（悬空，触发防跌落保护）
    bool floor_detected(uint8_t index) const;
    // 四路地检悬空位掩码：置 1 表示该路 floor_detected()==false（悬空，触发防跌落保护）
    static constexpr uint8_t kFloorLostLeft = 1u << 0;
    static constexpr uint8_t kFloorLostLeftMiddle = 1u << 1;
    static constexpr uint8_t kFloorLostRightMiddle = 1u << 2;
    static constexpr uint8_t kFloorLostRight = 1u << 3;
    static constexpr uint8_t kFloorLostAll = 0x0F;
    // 当前悬空快照，由轮询任务在每次 refresh() 后原子发布，供速度指令过滤读取
    uint8_t floor_lost_mask(void) const;
    // 头顶/下巴障碍物检测，低电平有效，返回 true 表示检测到障碍物（触发对应方向保护）
    bool head_up_obstacle(void) const;
    bool head_down_obstacle(void) const;
    bool int_active(void) const;
private:
    bool read_reg(uint8_t reg, uint8_t &value) const;
    bool write_reg(uint8_t reg, uint8_t value) const;
    bool apply_direction(void) const;
    // 由当前输入缓存计算四路悬空掩码，只在 refresh() 中调用
    uint8_t floor_lost_mask_from_cache(void) const;
    static void task(void *arg);

    bool _is_ready = false;
    Config _config;
    i2c_master_bus_handle_t _bus = nullptr;
    i2c_master_dev_handle_t _dev = nullptr;
    std::atomic<uint8_t> _input0{0};
    std::atomic<uint8_t> _input1{0};
    // 初值按"全部悬空"处理：I2C 尚未就绪或刷新失败时保守禁止前进
    std::atomic<uint8_t> _floor_lost_mask{kFloorLostAll};
    bool _fan_level = false;
    uint8_t _last_input0 = 0;
    uint8_t _last_input1 = 0;
    bool _first_refresh = true;
};

extern Xl9535Class Xl9535;

std::ostream &operator<<(std::ostream &os, const Xl9535Class &x);

// fmt v11 不再隐式使用 operator<< 作为兜底，需显式继承 ostream_formatter
template <> struct fmt::formatter<Xl9535Class> : fmt::ostream_formatter {};
