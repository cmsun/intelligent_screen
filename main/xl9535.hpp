#pragma once

#include <cstdint>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

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
    static constexpr Pin fan_pin = Pin::io1_1;             // 风扇控制，高电平转
    static constexpr Pin floor_left_pin = Pin::io1_2;      // 左地面检测
    static constexpr Pin floor_middle_pin = Pin::io1_3;    // 中地面检测
    static constexpr Pin floor_right_pin = Pin::io1_4;     // 右地面检测
    static constexpr Pin head_up_limit_pin = Pin::io0_6;   // 抬头限位，低有效
    static constexpr Pin head_down_limit_pin = Pin::io0_7; // 低头限位，低有效

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
    // 地面检测：index 0=左 1=中 2=右，高电平表示检测到地面
    bool floor_detected(uint8_t index) const;
    // 限位信号，低电平有效，返回 true 表示触发限位
    bool head_up_limited(void) const;
    bool head_down_limited(void) const;
    bool int_active(void) const;
    void print_status(void) const;

private:
    bool read_reg(uint8_t reg, uint8_t &value) const;
    bool write_reg(uint8_t reg, uint8_t value) const;
    bool apply_direction(void) const;
    static void task(void *arg);

    bool _is_ready = false;
    Config _config;
    i2c_master_bus_handle_t _bus = nullptr;
    i2c_master_dev_handle_t _dev = nullptr;
    uint8_t _input0 = 0;
    uint8_t _input1 = 0;
    bool _fan_level = false;
    uint8_t _last_input0 = 0;
    uint8_t _last_input1 = 0;
    bool _first_refresh = true;
};

extern Xl9535Class Xl9535;
