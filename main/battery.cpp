#include "battery.hpp"

#include <cmath>

#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "esplog.hpp"

BatteryClass Battery;

// ADC 校准句柄（按通道各创建一个，失败则退回线性换算）
static adc_cali_handle_t s_cali_bat_volt = nullptr;
static adc_cali_handle_t s_cali_bat_curr = nullptr;
static adc_cali_handle_t s_cali_charg_in = nullptr;

// io -> adc channel 映射，返回 false 表示该 IO 不属于指定 ADC 单元
static bool io_to_channel(gpio_num_t io, adc_unit_t unit, adc_channel_t &chan)
{
    adc_unit_t got_unit = ADC_UNIT_1;
    adc_channel_t got_chan = ADC_CHANNEL_0;
    if (adc_oneshot_io_to_channel(static_cast<int>(io), &got_unit, &got_chan) != ESP_OK)
    {
        return false;
    }
    if (got_unit != unit)
    {
        return false;
    }
    chan = got_chan;
    return true;
}

// 创建曲线拟合校准句柄，失败返回 nullptr
static adc_cali_handle_t create_cali(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten)
{
    adc_cali_handle_t handle = nullptr;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = unit,
        .chan = chan,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg, &handle) != ESP_OK)
    {
        return nullptr;
    }
#endif  // ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    return handle;
}

bool BatteryClass::begin(void)
{
    return begin(Config{});
}

bool BatteryClass::begin(const Config &config)
{
    _config = config;
    _is_ready = false;

    // 1. 充电使能管脚：默认关闭充电
    gpio_config_t en_cfg = {
        .pin_bit_mask = 1ULL << static_cast<uint32_t>(_config.charg_en_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&en_cfg) != ESP_OK)
    {
        esplog::error("charge enable gpio config failed");
        return false;
    }
    gpio_set_level(_config.charg_en_pin, 0);
    _status.charge_enabled = false;

    // 2. IO -> ADC channel 映射
    if (!io_to_channel(_config.bat_volt_pin, _config.adc_unit, _bat_volt_chan) ||
        !io_to_channel(_config.bat_curr_pin, _config.adc_unit, _bat_curr_chan) ||
        !io_to_channel(_config.charg_in_pin, _config.adc_unit, _charg_in_chan))
    {
        esplog::error("io to adc channel failed");
        return false;
    }

    // 3. 初始化 ADC oneshot 单元
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = _config.adc_unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &_adc_handle) != ESP_OK)
    {
        esplog::error("adc oneshot new unit failed");
        return false;
    }

    // 4. 配置三个采样通道
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = _config.adc_atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_channel_t chans[] = {_bat_volt_chan, _bat_curr_chan, _charg_in_chan};
    for (adc_channel_t chan : chans)
    {
        if (adc_oneshot_config_channel(_adc_handle, chan, &chan_cfg) != ESP_OK)
        {
            esplog::error("adc channel {} config failed", static_cast<int>(chan));
            return false;
        }
    }

    // 5. 创建校准句柄（失败不影响使用，退回线性换算）
    s_cali_bat_volt = create_cali(_config.adc_unit, _bat_volt_chan, _config.adc_atten);
    s_cali_bat_curr = create_cali(_config.adc_unit, _bat_curr_chan, _config.adc_atten);
    s_cali_charg_in = create_cali(_config.adc_unit, _charg_in_chan, _config.adc_atten);

    _is_ready = true;
    esplog::info("bms ready: bat_adc={} curr_adc={} charg_adc={}",
                 static_cast<int>(_bat_volt_chan), static_cast<int>(_bat_curr_chan),
                 static_cast<int>(_charg_in_chan));

    // 先采样一次，用开路电压初始化当前电量
    update();

    // 创建采样任务：周期采样并更新 SOC
    if (xTaskCreate(battery_task, "battery_task", _config.task_stack, nullptr, 4, nullptr) != pdPASS)
    {
        esplog::error("battery task create failed");
        return false;
    }
    return true;
}

void BatteryClass::battery_task(void *arg)
{
    while (true)
    {
        Battery.update();
        Battery._log_acc_ms += Battery._config.sample_period_ms;
        if (Battery._log_acc_ms >= Battery._config.log_period_ms)
        {
            Battery._log_acc_ms = 0;
            Battery.log_status();
        }
        vTaskDelay(pdMS_TO_TICKS(Battery._config.sample_period_ms));
    }
}

void BatteryClass::log_status(void)
{
    esplog::debug("battery {:.2f}V, charger {:.2f}V, current {:+.2f}A, power {:+.1f}W, soc {:.1f}%, charge_en {}, state {}",
                  _status.battery_voltage, _status.charger_voltage, _status.current,
                  _status.power, _status.soc, _status.charge_enabled ? "on" : "off", _status.state);
}

// 读取一次 ADC 并换算为毫伏，失败返回 false
bool BatteryClass::read_channel(adc_channel_t chan, int &out_mv) const
{
    if (!_is_ready)
    {
        return false;
    }
    int raw = 0;
    if (adc_oneshot_read(_adc_handle, chan, &raw) != ESP_OK)
    {
        return false;
    }
    int mv = 0;
    adc_cali_handle_t cali = nullptr;
    if (chan == _bat_volt_chan)
    {
        cali = s_cali_bat_volt;
    }
    else if (chan == _bat_curr_chan)
    {
        cali = s_cali_bat_curr;
    }
    else
    {
        cali = s_cali_charg_in;
    }
    if (cali != nullptr && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK)
    {
        out_mv = mv;
        return true;
    }
    // 无校准时按 12 位、参考电压 3.3V 线性换算
    out_mv = static_cast<int>(raw * 3300 / 4095);
    return true;
}

// 磷酸铁锂单节开路电压(OCV) -> SOC 曲线，静置电压查表线性插值
// 仅用于开机时初始化电量，运行中的电量变化由安时积分计算
float BatteryClass::estimate_soc(float voltage) const
{
    static constexpr float kCellVolt[] = {2.50f, 2.80f, 3.00f, 3.15f, 3.20f, 3.22f,
                                          3.25f, 3.26f, 3.28f, 3.30f, 3.32f, 3.35f, 3.65f};
    static constexpr float kSoc[] = {0.0f, 1.0f, 5.0f, 10.0f, 20.0f, 30.0f,
                                     40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f};
    static constexpr size_t kPointCount = sizeof(kCellVolt) / sizeof(kCellVolt[0]);

    const float cell_v = voltage / _config.cell_count;
    if (cell_v <= kCellVolt[0])
    {
        return 0.0f;
    }
    if (cell_v >= kCellVolt[kPointCount - 1])
    {
        return 100.0f;
    }
    for (size_t i = 1; i < kPointCount; ++i)
    {
        if (cell_v <= kCellVolt[i])
        {
            const float ratio = (cell_v - kCellVolt[i - 1]) / (kCellVolt[i] - kCellVolt[i - 1]);
            return kSoc[i - 1] + ratio * (kSoc[i] - kSoc[i - 1]);
        }
    }
    return 0.0f;
}

// 安时积分：dSOC = I * dt / 容量，电流为正表示充电
void BatteryClass::integrate_soc(float current_a, float dt_hours)
{
    const float delta_soc = current_a * dt_hours / _config.capacity_ah * 100.0f;
    _soc_integral += delta_soc;
    // 积分只能到 soc_integral_max，剩下部分由电压判定充满后补到 100%
    if (_soc_integral > _config.soc_integral_max)
    {
        _soc_integral = _config.soc_integral_max;
    }
    // 放电时只能降到 0%，不允许为负
    if (_soc_integral < 0.0f)
    {
        _soc_integral = 0.0f;
    }
}

void BatteryClass::set_charge_enable(bool enable)
{
    if (!_is_ready)
    {
        return;
    }
    gpio_set_level(_config.charg_en_pin, enable ? 1 : 0);
    _status.charge_enabled = enable;
}

bool BatteryClass::charge_enabled(void) const
{
    return _status.charge_enabled;
}

void BatteryClass::update(void)
{
    if (!_is_ready)
    {
        return;
    }
    int mv = 0;
    // 1. 电池电压：ADC 电压 / 分压比
    if (read_channel(_bat_volt_chan, mv))
    {
        const float ratio = _config.bat_divider_low / (_config.bat_divider_up + _config.bat_divider_low);
        _status.battery_voltage = (mv / 1000.0f) / ratio;
    }
    // 2. 充电器电压
    if (read_channel(_charg_in_chan, mv))
    {
        const float ratio = _config.charg_divider_low / (_config.charg_divider_up + _config.charg_divider_low);
        _status.charger_voltage = (mv / 1000.0f) / ratio;
    }
    // 3. 电流：I = (Vout - Vref) / (gain * Rsense)，正为充电
    if (read_channel(_bat_curr_chan, mv))
    {
        const float vout = mv / 1000.0f;
        _status.current = (vout - _config.ina_v_ref) / (_config.ina_gain * _config.ina_r_sense);
        esplog::error("vout:{} current: {}", vout, _status.current);
    }
    _status.power = _status.battery_voltage * _status.current;

    // 4. 电量计算
    const int64_t now_us = esp_timer_get_time();
    if (!_soc_initialized)
    {
        // 开机首次：用开路电压初始化当前电量
        _soc_integral = estimate_soc(_status.battery_voltage);
        _soc_initialized = true;
        esplog::info("soc initialized by voltage: {:.1f}% (battery {:.2f}V)",
                     _soc_integral, _status.battery_voltage);
    }
    else
    {
        const float dt_hours = static_cast<float>(now_us - _last_update_us) / 3.6e9f;
        integrate_soc(_status.current, dt_hours);
    }
    _last_update_us = now_us;

    update_state();
    // update_state 中可能因充满/亏电修正积分电量，最后再同步一次
    _status.soc = _soc_integral;
}

void BatteryClass::update_state(void)
{
    const float cur = _status.current;
    const float volt = _status.battery_voltage;
    const float cell_v = volt / _config.cell_count;
    // 充电器电压 >= 19V 认为充电器接入，< 19V 认为已拔除
    const bool charger_on = _status.charger_voltage >= _config.charger_present_v;

    // 1. 过流保护：立即断开充电
    if (std::fabs(cur) > _config.over_current)
    {
        _status.state = State::over_current;
        set_charge_enable(false);
        return;
    }

    // 2. 充满判定：充电使能 && (电压 >= 29V || (电压 >= 28V && 充电电流 < 0.5A))
    if (!_is_full && _status.charge_enabled &&
        (volt >= _config.full_voltage ||
         (volt >= _config.full_voltage_min && cur < _config.full_current_max)))
    {
        _is_full = true;
        esplog::info("battery full: {:.2f}V, current {:+.2f}A", volt, cur);
    }
    // 已充满且充电器仍在线时保持充电关闭，拔除充电器后清除充满标志
    if (_is_full)
    {
        if (!charger_on)
        {
            _is_full = false;
        }
        else
        {
            _soc_integral = 100.0f;
            _status.state = State::full;
            set_charge_enable(false);
            return;
        }
    }

    // 3. 电压低于亏电阈值时，积分电量强制归零（不允许负数）
    if (cell_v <= _config.cell_empty_v)
    {
        _soc_integral = 0.0f;
    }
    if (cell_v <= _config.cell_low_warn_v)
    {
        _status.state = State::low_battery;
        // 充电器在位时才允许充电
        set_charge_enable(charger_on);
        return;
    }

    // 4. 充电器在位且未充满则使能充电，拔除充电器则失能充电
    set_charge_enable(charger_on);

    // 5. 充放电/静置状态
    if (cur > _config.idle_current)
    {
        _status.state = State::charging;
    }
    else if (cur < -_config.idle_current)
    {
        _status.state = State::discharging;
    }
    else
    {
        _status.state = State::idle;
    }
}

BatteryClass::Status BatteryClass::status(void) const
{
    return _status;
}

float BatteryClass::battery_voltage(void) const
{
    return _status.battery_voltage;
}

float BatteryClass::charger_voltage(void) const
{
    return _status.charger_voltage;
}

float BatteryClass::current(void) const
{
    return _status.current;
}

float BatteryClass::soc(void) const
{
    return _status.soc;
}

BatteryClass::State BatteryClass::state(void) const
{
    return _status.state;
}

std::ostream &operator<<(std::ostream &os, const BatteryClass &b)
{
    const auto &s = b._status;
    os << "battery " << s.battery_voltage << "V, "
       << "charger " << s.charger_voltage << "V, "
       << "current " << s.current << "A, "
       << "power " << s.power << "W, "
       << "soc " << s.soc << "%, "
       << "charge_en " << (s.charge_enabled ? "on" : "off") << ", "
       << "state " << s.state;
    return os;
}

