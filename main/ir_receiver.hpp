#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/rmt_rx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

/// HS-S23P 是 38 kHz 一体化红外接收头：片内已完成解调，S 脚输出的是基带脉冲
/// （空闲高电平、收到载波时低电平），所以必须按脉冲宽度解码，不能当普通电平读。
/// 本类用两个 RMT RX 通道采集边沿时序并解码 NEC 协议：
///   左接收头 -> GPIO13，右接收头 -> GPIO14。
/// 两路共用一个解码任务；要「遥控器按了什么键」用 last_frame()/read()，
/// 要「红外源在左还是右」用 last_rx_ms()/signal_seen()。

enum class IrSide : uint8_t
{
    Left = 0,
    Right = 1,
};

/// 一帧 NEC 解码结果
struct IrFrame
{
    IrSide side = IrSide::Left; ///< 来自哪一侧
    uint32_t raw_code = 0;      ///< 32 位原始码（LSB first：地址/反地址/命令/反命令）
    uint8_t address = 0;        ///< 标准 NEC 8 位地址
    uint8_t command = 0;        ///< 标准 NEC 8 位命令
    bool address_valid = false; ///< 地址与命令的反码校验通过（扩展 NEC 可能为 false）
    bool repeat = false;        ///< 长按重复帧（本身不含数据，沿用上一帧）
    uint32_t timestamp_ms = 0;  ///< 收到时刻（esp_timer 毫秒）
    size_t symbol_count = 0;    ///< RMT 符号数，便于排查非 NEC 帧
};

class IrReceiverClass
{
public:
    struct Config
    {
        gpio_num_t left_pin = GPIO_NUM_13;  ///< 左红外接收器 S 脚
        gpio_num_t right_pin = GPIO_NUM_14; ///< 右红外接收器 S 脚
        uint32_t resolution_hz = 1'000'000;        ///< RMT 分辨率，1 tick = 1 us
        uint32_t signal_range_min_ns = 1250;       ///< 短于此宽度的脉冲视为毛刺
        uint32_t signal_range_max_ns = 12'000'000; ///< 空闲超过此时长判定一帧结束
        uint32_t decode_margin_us = 300;           ///< NEC 时序容差（遥控器偏差）
        bool invert_input = false;  ///< 模块输出若与标准反相（空闲低）置 true
        uint32_t task_stack_bytes = 4096;
        UBaseType_t task_priority = 4;
        bool log_frames = true; ///< 解出有效帧时打印日志
    };

    static constexpr size_t kChannelCount = 2;
    /// ESP32-S3 每个 RMT 通道 48 个 symbol word，标准 NEC 帧 34 个。
    /// 取 48 正好占满一个通道，不会挤占相邻通道的内存块。
    static constexpr size_t kMaxSymbols = 48;

    IrReceiverClass() = default;
    ~IrReceiverClass();

    IrReceiverClass(const IrReceiverClass &) = delete;
    IrReceiverClass &operator=(const IrReceiverClass &) = delete;

    /// 初始化两路 RMT 接收并启动解码任务（与 Xl9535.begin() 风格一致）
    bool begin(void);
    bool begin(const Config &config);

    [[nodiscard]] bool is_ready() const noexcept { return _ready; }

    /// 最新一帧快照（不清空）；尚未收到帧时 timestamp_ms == 0
    [[nodiscard]] IrFrame last_frame() const;
    [[nodiscard]] IrFrame last_frame(IrSide side) const;

    /// 消费式读取：取出新帧后清标记，返回 false 表示没有新帧
    bool read(IrFrame &out);
    bool read(IrSide side, IrFrame &out);

    /// 指定侧最后一次「有红外活动」的时刻（ms），解不出 NEC 的帧也算活动
    [[nodiscard]] uint32_t last_rx_ms(IrSide side) const;
    /// 最近 within_ms 内该侧是否收到过红外信号（用于左右方位判断）
    [[nodiscard]] bool signal_seen(IrSide side, uint32_t within_ms) const;

private:
    struct Channel
    {
        rmt_channel_handle_t handle = nullptr;
        std::array<rmt_symbol_word_t, kMaxSymbols> symbols{}; ///< 供 RMT 写入的持久缓冲
        IrFrame frame{};
        uint32_t last_activity_ms = 0;
    };

    /// 每个通道注册回调时传入的上下文，必须是持久内存（ISR 中使用）
    struct RxContext
    {
        QueueHandle_t queue = nullptr;
        size_t channel_index = 0;
    };

    struct RxEvent
    {
        size_t channel_index = 0;
        rmt_rx_done_event_data_t data{};
    };

    static bool rx_done_callback(rmt_channel_handle_t channel,
                                 const rmt_rx_done_event_data_t *edata,
                                 void *user_ctx);
    static void decode_task(void *arg);

    void on_frame(const RxEvent &event);
    [[nodiscard]] bool parse_nec(const rmt_symbol_word_t *symbols, IrFrame &out) const;
    [[nodiscard]] bool duration_matches(uint32_t ticks, uint32_t expected_us) const noexcept;
    void cleanup() noexcept;
    static size_t side_index(IrSide side) noexcept;
    static const char *side_name(IrSide side) noexcept;

    Config _cfg{};
    std::array<Channel, kChannelCount> _channels{};
    std::array<RxContext, kChannelCount> _contexts{};
    QueueHandle_t _queue = nullptr;
    TaskHandle_t _task = nullptr;
    IrFrame _last_frame{};
    std::atomic<uint32_t> _pending_mask{0};
    bool _ready = false;
};

/// 全局红外接收实例
extern IrReceiverClass IrReceiver;
