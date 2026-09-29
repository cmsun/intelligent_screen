#pragma once

#include <cstdint>
#include <ostream>

// fmtlib 自定义类型的字符串格式化需包含 <fmt/ostream.h>
#include <fmt/ostream.h>

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>

///
/// 电池管理系统（BMS）
///
/// 接线（ESP32-S3 侧，见 doc/AgvmainBoard.pdf）：
///   GPIO1 -> CHARG_IN  充电器电压采样（分压后接 ADC）
///   GPIO3 -> CHARG_EN  充电使能（输出）
///   GPIO4 -> BAT_Vol   电池电压采样（分压后接 ADC）
///   GPIO5 -> BAT_CIURR INA240 电流检测输出（接 ADC）
///
class BatteryClass
{
public:
    struct Config
    {
        gpio_num_t charg_in_pin = GPIO_NUM_1;  // CHARG_IN -> ADC1_CH0
        gpio_num_t charg_en_pin = GPIO_NUM_3;  // CHARG_EN -> 输出
        gpio_num_t bat_volt_pin = GPIO_NUM_4;  // BAT_Vol -> ADC1_CH3
        gpio_num_t bat_curr_pin = GPIO_NUM_5;  // BAT_CIURR -> ADC1_CH4
        adc_unit_t adc_unit = ADC_UNIT_1;
        adc_atten_t adc_atten = ADC_ATTEN_DB_12;  // 量程约 0~3.1V
        // 电压采样分压：V_adc = V_in * r_low / (r_up + r_low)
        float bat_divider_up = 18.0f;     // kΩ，电池分压上臂（PIR120）
        float bat_divider_low = 2.0f;     // kΩ，电池分压下臂
        float charg_divider_up = 18.0f;   // kΩ，充电器分压上臂
        float charg_divider_low = 2.0f;   // kΩ，充电器分压下臂
        // INA240 参数：I = (Vout - Vref) / (gain * r_sense)
        float ina_gain = 20.0f;        // INA240A1 增益，按实际型号修改
        float ina_r_sense = 0.005f;      // Ω，采样电阻阻值
        float ina_v_ref = 1.65f;       // V，双向接法 REF = VS/2
        float ina_v_supply = 3.3f;     // V，INA240 供电电压
        uint32_t sample_period_ms = 200;   // 采样周期
        uint32_t log_period_ms = 2000;     // 日志打印周期
        // 6 串锂电池（29.2V 满电）阈值，按实际电池组修改
        float cell_count = 8.0f;           // 串联节数
        float cell_full_v = 3.65f;          // 单节满电电压
        float cell_empty_v = 2.5f;         // 单节亏电电压
        float cell_low_warn_v = 3.0f;      // 单节低压告警电压
        float over_current = 8.0f;         // A，过流保护阈值
        // 安时积分（库仑计）参数
        float capacity_ah = 6.0f;          // Ah，电池额定容量
        float soc_integral_max = 99.0f;    // %，积分上限，剩余部分由电压判定充满
        // 充满判定：充电使能 && (电压 >= full_voltage ||
        //                       (电压 >= full_voltage_min && 充电电流 < full_current_max))
        float full_voltage = 29.0f;        // V，直接判定充满的电压
        float full_voltage_min = 28.0f;    // V，配合小电流判定充满的电压
        float full_current_max = 0.5f;     // A，判定充满的电流阈值
        float idle_current = 0.2f;         // A，|I| 小于该值视为静置（充放电判定用）
        // 充电器在位判定：充电器电压 >= 该值认为充电器接入
        float charger_present_v = 19.0f;   // V
        uint32_t task_stack = 1024 * 3;    // 采样任务栈大小（字节）
    };

    enum class State
    {
        idle = 0,     // 未充电也未放电
        discharging,  // 放电中
        charging,     // 充电中
        full,         // 已充满
        low_battery,  // 电量过低
        over_current, // 过流保护
    };

    struct Status
    {
        float battery_voltage = 0.0f;  // V，电池电压
        float charger_voltage = 0.0f;  // V，充电器电压
        float current = 0.0f;          // A，正为充电、负为放电
        float power = 0.0f;            // W，正为充电功率
        float soc = 0.0f;              // %，估算电量
        State state = State::idle;
        bool charge_enabled = false;   // CHARG_EN 当前电平
    };

    BatteryClass(void) = default;
    ~BatteryClass(void) = default;

    bool begin(void);
    bool begin(const Config &config);
    void set_charge_enable(bool enable);
    bool charge_enabled(void) const;
    // 采样一次并更新内部状态
    void update(void);
    Status status(void) const;
    float battery_voltage(void) const;
    float charger_voltage(void) const;
    float current(void) const;
    float soc(void) const;
    State state(void) const;
    friend std::ostream &operator<<(std::ostream &os, const BatteryClass &b);

private:
    bool _is_ready = false;
    Config _config;
    Status _status;
    adc_oneshot_unit_handle_t _adc_handle = nullptr;
    adc_channel_t _bat_volt_chan = ADC_CHANNEL_3;
    adc_channel_t _bat_curr_chan = ADC_CHANNEL_4;
    adc_channel_t _charg_in_chan = ADC_CHANNEL_0;
    bool read_channel(adc_channel_t chan, int &out_mv) const;
    // 开路电压法估算 SOC（磷酸铁锂 OCV 曲线插值），仅用于开机初始化
    float estimate_soc(float voltage) const;
    void update_state(void);
    // 安时积分：按电流与时间更新积分电量，并做上下限约束
    void integrate_soc(float current_a, float dt_hours);

    float _soc_integral = 0.0f;    // %，安时积分得到的电量
    bool _soc_initialized = false; // 开机是否已用电压初始化过电量
    int64_t _last_update_us = 0;   // 上次积分时间戳（us）
    uint32_t _log_acc_ms = 0;      // 日志打印计时
    bool _is_full = false;         // 充满标志，充电器拔除后清除
    static void battery_task(void *arg);
    void log_status(void);
};

constexpr std::ostream &operator<<(std::ostream &os, const BatteryClass::State &state)
{
    switch (state)
    {
    case BatteryClass::State::idle:
        return os << "idle";
    case BatteryClass::State::discharging:
        return os << "discharging";
    case BatteryClass::State::charging:
        return os << "charging";
    case BatteryClass::State::full:
        return os << "full";
    case BatteryClass::State::low_battery:
        return os << "low_battery";
    case BatteryClass::State::over_current:
        return os << "over_current";
    default:
        return os << "unknown state";
    }
}

extern BatteryClass Battery;

// fmt v11 不再隐式使用 operator<< 作为兜底，需显式继承 ostream_formatter
template <> struct fmt::formatter<BatteryClass::State> : fmt::ostream_formatter {};
template <> struct fmt::formatter<BatteryClass> : fmt::ostream_formatter {};
