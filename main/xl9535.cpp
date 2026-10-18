#include "xl9535.hpp"

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esplog.hpp"

Xl9535Class Xl9535;

// 单个管脚所属的端口（0/1）与端口内位掩码
static uint8_t pin_port(Xl9535Class::Pin pin)
{
    return static_cast<uint8_t>(pin) < 8 ? 0 : 1;
}

static uint8_t pin_mask(Xl9535Class::Pin pin)
{
    return static_cast<uint8_t>(1u << (static_cast<uint8_t>(pin) & 0x07u));
}

bool Xl9535Class::begin(void)
{
    return begin(Config{});
}

bool Xl9535Class::begin(const Config &config)
{
    _config = config;
    _is_ready = false;

    // INT 引脚配置为输入带上拉（芯片侧为开漏输出，低电平有效）
    gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << static_cast<uint32_t>(_config.int_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&int_cfg) != ESP_OK)
    {
        esplog::error("int gpio config failed");
        return false;
    }

    // 创建 I2C 主机总线
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = _config.sda_pin,
        .scl_io_num = _config.scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
    };
    if (i2c_new_master_bus(&bus_cfg, &_bus) != ESP_OK)
    {
        esplog::error("i2c new master bus failed");
        return false;
    }

    // 探测总线：XL9535 的 A0/A1/A2 决定地址，落在 0x20~0x27 范围内
    bool probed = false;
    for (uint16_t addr = 0x20; addr <= 0x27; ++addr)
    {
        const esp_err_t err = i2c_master_probe(_bus, addr, _config.i2c_timeout_ms);
        if (err == ESP_OK)
        {
            probed = true;
            esplog::info("xl9535 probe ack at 0x{:02X}", addr);
        }
    }
    if (!probed)
    {
        esplog::error("xl9535 probe: no ack in 0x20~0x27, scl {} sda {}",
                      static_cast<int>(_config.scl_pin), static_cast<int>(_config.sda_pin));
    }

    // 挂载 XL9535 设备
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _config.device_address,
        .scl_speed_hz = _config.i2c_freq_hz,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };
    if (i2c_master_bus_add_device(_bus, &dev_cfg, &_dev) != ESP_OK)
    {
        esplog::error("xl9535 add device failed, addr 0x{:02X}", _config.device_address);
        return false;
    }

    // 配置方向：除风扇（IO1_1）为输出外，其余均为输入
    if (!apply_direction())
    {
        esplog::error("xl9535 apply direction failed");
        return false;
    }
    // 风扇初始为停止
    if (!set_fan(false))
    {
        esplog::error("xl9535 set fan failed");
        return false;
    }
    if (!refresh())
    {
        esplog::error("xl9535 refresh failed");
        return false;
    }

    _is_ready = true;
    esplog::info("xl9535 ready, addr 0x{:02X}, scl {}, sda {}, int {}",
                 _config.device_address, static_cast<int>(_config.scl_pin),
                 static_cast<int>(_config.sda_pin), static_cast<int>(_config.int_pin));

    print_status();

    // 创建轮询任务：周期刷新输入并在状态变化时打印日志
    if (xTaskCreate(task, "xl9535_task", 1024 * 5, nullptr, 4, nullptr) != pdPASS)
    {
        esplog::error("xl9535 task create failed");
        return false;
    }
    return true;
}

// 除风扇管脚外全部设为输入：配置寄存器位为 1 表示输入
bool Xl9535Class::apply_direction(void) const
{
    const uint8_t dir0 = 0xFF;                  // IO0_x 全部输入
    const uint8_t dir1 = 0xFF & ~pin_mask(fan_pin);  // IO1_1 输出，其余输入
    return write_reg(0x06, dir0) && write_reg(0x07, dir1);
}

bool Xl9535Class::read_reg(uint8_t reg, uint8_t &value) const
{
    const esp_err_t err =
        i2c_master_transmit_receive(_dev, &reg, 1, &value, 1, _config.i2c_timeout_ms);
    if (err != ESP_OK)
    {
        esplog::error("xl9535 read reg 0x{:02X} failed: {}", reg, esp_err_to_name(err));
    }
    return err == ESP_OK;
}

bool Xl9535Class::write_reg(uint8_t reg, uint8_t value) const
{
    uint8_t data[2] = {reg, value};
    const esp_err_t err = i2c_master_transmit(_dev, data, sizeof(data), _config.i2c_timeout_ms);
    if (err != ESP_OK)
    {
        esplog::error("xl9535 write reg 0x{:02X} = 0x{:02X} failed: {}",
                      reg, value, esp_err_to_name(err));
    }
    return err == ESP_OK;
}

// 一次连读两个输入端口（0x00 起地址自增）
bool Xl9535Class::refresh(void)
{
    uint8_t reg = 0x00;
    uint8_t buf[2] = {0};
    const esp_err_t err =
        i2c_master_transmit_receive(_dev, &reg, 1, buf, sizeof(buf), _config.i2c_timeout_ms);
    if (err != ESP_OK)
    {
        esplog::error("xl9535 read input failed: {}", esp_err_to_name(err));
        return false;
    }
    _input0 = buf[0];
    _input1 = buf[1];
    return true;
}

bool Xl9535Class::set_pin(Pin pin, bool level)
{
    uint8_t value = 0;
    const uint8_t reg = pin_port(pin) == 0 ? 0x02 : 0x03;
    if (!read_reg(reg, value))
    {
        return false;
    }
    if (level)
    {
        value |= pin_mask(pin);
    }
    else
    {
        value &= static_cast<uint8_t>(~pin_mask(pin));
    }
    if (!write_reg(reg, value))
    {
        esplog::error("xl9535 set pin failed");
        return false;
    }
    return true;
}

bool Xl9535Class::read_pin(Pin pin, bool &level) const
{
    const uint8_t value = pin_port(pin) == 0 ? _input0 : _input1;
    level = (value & pin_mask(pin)) != 0;
    return true;
}

bool Xl9535Class::set_fan(bool on)
{
    if (!set_pin(fan_pin, on))
    {
        return false;
    }
    _fan_level = on;
    esplog::info("fan {}", on ? "on" : "off");
    return true;
}

bool Xl9535Class::fan_on(void) const
{
    return _fan_level;
}

bool Xl9535Class::floor_detected(uint8_t index) const
{
    switch (index)
    {
    case 0:
        return (_input1 & pin_mask(floor_left_pin)) != 0;
    case 1:
        return (_input1 & pin_mask(floor_middle_pin)) != 0;
    case 2:
        return (_input1 & pin_mask(floor_right_pin)) != 0;
    default:
        return false;
    }
}

// 限位信号为低电平有效，读到 0 表示触发限位
bool Xl9535Class::head_up_limited(void) const
{
    return (_input0 & pin_mask(head_up_limit_pin)) == 0;
}

bool Xl9535Class::head_down_limited(void) const
{
    return (_input0 & pin_mask(head_down_limit_pin)) == 0;
}

bool Xl9535Class::int_active(void) const
{
    return gpio_get_level(_config.int_pin) == 0;
}

void Xl9535Class::print_status(void) const
{
    esplog::info("floor left {} middle {} right {}, limit up {} down {}, fan {}",
                 floor_detected(0) ? "detected" : "lost",
                 floor_detected(1) ? "detected" : "lost",
                 floor_detected(2) ? "detected" : "lost",
                 head_up_limited() ? "triggered" : "idle",
                 head_down_limited() ? "triggered" : "idle",
                 _fan_level ? "on" : "off");
}

void Xl9535Class::task(void *arg)
{
    while (true)
    {
        Xl9535.refresh();
        // 输入端口有变化时才打印日志，避免刷屏
        if (!Xl9535._first_refresh &&
            (Xl9535._input0 != Xl9535._last_input0 || Xl9535._input1 != Xl9535._last_input1))
        {
            Xl9535.print_status();
        }
        Xl9535._first_refresh = false;
        Xl9535._last_input0 = Xl9535._input0;
        Xl9535._last_input1 = Xl9535._input1;
        vTaskDelay(pdMS_TO_TICKS(Xl9535._config.poll_period_ms));
    }
}
