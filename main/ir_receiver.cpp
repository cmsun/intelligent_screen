#include "ir_receiver.hpp"

#include <esp_err.h>
#include <esp_timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstdint>

#include "esplog.hpp"

namespace
{
// NEC 标称时序（us）；duration_matches 会按 resolution_hz 把 RMT tick 换算后比较
constexpr uint32_t kLeadLowUs = 9000;
constexpr uint32_t kLeadHighUs = 4500;
constexpr uint32_t kRepeatHighUs = 2250;
constexpr uint32_t kBitLowUs = 560;
constexpr uint32_t kBitZeroHighUs = 560;
constexpr uint32_t kBitOneHighUs = 1690;
constexpr size_t kNecDataBits = 32;
// leader + 32 个数据位；末尾的 stop 位可有可无（不同遥控器帧长 33 或 34）
constexpr size_t kNecFrameMinSymbols = 1 + kNecDataBits;
constexpr size_t kNecRepeatSymbols = 2;

constexpr IrSide kSides[2] = {IrSide::Left, IrSide::Right};

uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
} // namespace

IrReceiverClass IrReceiver;

IrReceiverClass::~IrReceiverClass()
{
    cleanup();
}

size_t IrReceiverClass::side_index(IrSide side) noexcept
{
    return side == IrSide::Right ? 1U : 0U;
}

const char *IrReceiverClass::side_name(IrSide side) noexcept
{
    return side == IrSide::Right ? "right" : "left";
}

void IrReceiverClass::cleanup() noexcept
{
    // 先停通道，避免回调继续往队列里塞事件；再删任务、删队列
    for (auto &ch : _channels)
    {
        if (ch.handle != nullptr)
        {
            rmt_disable(ch.handle);
            rmt_del_channel(ch.handle);
            ch.handle = nullptr;
        }
    }
    if (_task != nullptr)
    {
        vTaskDelete(_task);
        _task = nullptr;
    }
    if (_queue != nullptr)
    {
        vQueueDelete(_queue);
        _queue = nullptr;
    }
    _ready = false;
}

bool IrReceiverClass::begin(void)
{
    return begin(Config{});
}

bool IrReceiverClass::begin(const Config &config)
{
    if (_ready)
        return true;

    _cfg = config;
    cleanup(); // 清掉上次失败可能留下的句柄

    _queue = xQueueCreate(8, sizeof(RxEvent));
    if (_queue == nullptr)
    {
        esplog::error("ir: create rx queue failed");
        return false;
    }

    const gpio_num_t pins[kChannelCount] = {_cfg.left_pin, _cfg.right_pin};
    for (size_t i = 0; i < kChannelCount; ++i)
    {
        rmt_rx_channel_config_t ch_cfg{};
        ch_cfg.gpio_num = pins[i];
        ch_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        ch_cfg.resolution_hz = _cfg.resolution_hz;
        ch_cfg.mem_block_symbols = kMaxSymbols;
        ch_cfg.intr_priority = 0;
        ch_cfg.flags.invert_in = _cfg.invert_input ? 1U : 0U;
        ch_cfg.flags.with_dma = 0;
        ch_cfg.flags.allow_pd = 0;

        esp_err_t err = rmt_new_rx_channel(&ch_cfg, &_channels[i].handle);
        if (err != ESP_OK)
        {
            esplog::error("ir {} rmt channel failed: {}", side_name(kSides[i]),
                          esp_err_to_name(err));
            cleanup();
            return false;
        }

        _contexts[i].queue = _queue;
        _contexts[i].channel_index = i;
        rmt_rx_event_callbacks_t cbs{};
        cbs.on_recv_done = &IrReceiverClass::rx_done_callback;
        err = rmt_rx_register_event_callbacks(_channels[i].handle, &cbs, &_contexts[i]);
        if (err == ESP_OK)
            err = rmt_enable(_channels[i].handle);
        if (err != ESP_OK)
        {
            esplog::error("ir {} rmt enable failed: {}", side_name(kSides[i]),
                          esp_err_to_name(err));
            cleanup();
            return false;
        }

        rmt_receive_config_t rx_cfg{};
        rx_cfg.signal_range_min_ns = _cfg.signal_range_min_ns;
        rx_cfg.signal_range_max_ns = _cfg.signal_range_max_ns;
        err = rmt_receive(_channels[i].handle, _channels[i].symbols.data(),
                          _channels[i].symbols.size() * sizeof(rmt_symbol_word_t), &rx_cfg);
        if (err != ESP_OK)
        {
            esplog::error("ir {} rmt receive failed: {}", side_name(kSides[i]),
                          esp_err_to_name(err));
            cleanup();
            return false;
        }
    }

    if (xTaskCreate(&IrReceiverClass::decode_task, "ir_rx", _cfg.task_stack_bytes, this,
                    _cfg.task_priority, &_task) != pdPASS)
    {
        esplog::error("ir: create decode task failed");
        cleanup();
        return false;
    }

    _ready = true;
    esplog::info("ir receiver ready: left GPIO{} right GPIO{}, {} Hz",
                 static_cast<int>(_cfg.left_pin), static_cast<int>(_cfg.right_pin),
                 _cfg.resolution_hz);
    return true;
}

bool IrReceiverClass::rx_done_callback(rmt_channel_handle_t,
                                       const rmt_rx_done_event_data_t *edata,
                                       void *user_ctx)
{
    auto *ctx = static_cast<RxContext *>(user_ctx);
    if (ctx == nullptr || ctx->queue == nullptr || edata == nullptr)
        return false;

    RxEvent event{};
    event.channel_index = ctx->channel_index;
    event.data = *edata; // 指针指向 Channel::symbols，是持久缓冲，出回调仍有效
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(ctx->queue, &event, &woken);
    return woken == pdTRUE;
}

void IrReceiverClass::decode_task(void *arg)
{
    auto *self = static_cast<IrReceiverClass *>(arg);
    RxEvent event{};
    while (true)
    {
        if (xQueueReceive(self->_queue, &event, portMAX_DELAY) != pdTRUE)
            continue;

        const size_t idx = event.channel_index;
        if (idx >= kChannelCount)
            continue;

        self->on_frame(event);

        // 一次 rmt_receive 只收一帧，处理完必须重新武装该通道
        rmt_receive_config_t rx_cfg{};
        rx_cfg.signal_range_min_ns = self->_cfg.signal_range_min_ns;
        rx_cfg.signal_range_max_ns = self->_cfg.signal_range_max_ns;
        rmt_receive(self->_channels[idx].handle, self->_channels[idx].symbols.data(),
                    self->_channels[idx].symbols.size() * sizeof(rmt_symbol_word_t), &rx_cfg);
    }
}

void IrReceiverClass::on_frame(const RxEvent &event)
{
    const size_t idx = event.channel_index;
    Channel &ch = _channels[idx];
    const IrSide side = kSides[idx];
    const uint32_t stamp = now_ms();
    const size_t count = event.data.num_symbols;
    const rmt_symbol_word_t *symbols = event.data.received_symbols;

    // 只要 RMT 采到一帧就说明这侧「看到」了红外，哪怕解不出 NEC
    ch.last_activity_ms = stamp;

    IrFrame frame{};
    frame.side = side;
    frame.timestamp_ms = stamp;
    frame.symbol_count = count;

    bool decoded = false;
    if (symbols != nullptr && count >= kNecFrameMinSymbols)
        decoded = parse_nec(symbols, frame);

    if (!decoded && symbols != nullptr && count == kNecRepeatSymbols &&
        duration_matches(symbols[0].duration0, kLeadLowUs) &&
        duration_matches(symbols[0].duration1, kRepeatHighUs))
    {
        // 重复帧不带数据，沿用该侧上一帧的码
        frame.raw_code = ch.frame.raw_code;
        frame.address = ch.frame.address;
        frame.command = ch.frame.command;
        frame.address_valid = ch.frame.address_valid;
        frame.repeat = true;
        decoded = ch.frame.timestamp_ms != 0;
    }

    if (!decoded)
    {
        esplog::debug("ir {} unknown frame ({} symbols)", side_name(side), count);
        return;
    }

    ch.frame = frame;
    _last_frame = frame;
    _pending_mask.fetch_or(1U << idx, std::memory_order_release);

    if (_cfg.log_frames)
    {
        esplog::debug("ir {} code=0x{:08X} addr=0x{:02X} cmd=0x{:02X}{}",
                      side_name(side), frame.raw_code, frame.address, frame.command,
                      frame.repeat ? " repeat" : "");
    }
}

bool IrReceiverClass::parse_nec(const rmt_symbol_word_t *symbols, IrFrame &out) const
{
    // 标准接收头空闲为高，第一个 symbol 应是 9ms 低 + 4.5ms 高的引导码
    if (!duration_matches(symbols[0].duration0, kLeadLowUs) ||
        !duration_matches(symbols[0].duration1, kLeadHighUs))
        return false;

    uint32_t raw = 0;
    for (size_t i = 0; i < kNecDataBits; ++i)
    {
        const rmt_symbol_word_t &s = symbols[1 + i];
        if (!duration_matches(s.duration0, kBitLowUs))
            return false;
        if (duration_matches(s.duration1, kBitOneHighUs))
            raw |= (1U << i); // NEC 低位先发
        else if (!duration_matches(s.duration1, kBitZeroHighUs))
            return false;
    }

    out.raw_code = raw;
    out.address = static_cast<uint8_t>(raw & 0xFFU);
    out.command = static_cast<uint8_t>((raw >> 16) & 0xFFU);
    const uint8_t addr_inv = static_cast<uint8_t>((raw >> 8) & 0xFFU);
    const uint8_t cmd_inv = static_cast<uint8_t>((raw >> 24) & 0xFFU);
    out.address_valid = (static_cast<uint8_t>(out.address ^ addr_inv) == 0xFFU) &&
                        (static_cast<uint8_t>(out.command ^ cmd_inv) == 0xFFU);
    out.repeat = false;
    return true;
}

bool IrReceiverClass::duration_matches(uint32_t ticks, uint32_t expected_us) const noexcept
{
    if (_cfg.resolution_hz == 0)
        return false;
    const uint64_t us = static_cast<uint64_t>(ticks) * 1'000'000ULL / _cfg.resolution_hz;
    const uint32_t low =
        expected_us > _cfg.decode_margin_us ? expected_us - _cfg.decode_margin_us : 0U;
    return us >= low && us <= static_cast<uint64_t>(expected_us + _cfg.decode_margin_us);
}

IrFrame IrReceiverClass::last_frame() const
{
    return _last_frame;
}

IrFrame IrReceiverClass::last_frame(IrSide side) const
{
    return _channels[side_index(side)].frame;
}

bool IrReceiverClass::read(IrFrame &out)
{
    const uint32_t mask = _pending_mask.exchange(0U, std::memory_order_acquire);
    if (mask == 0U)
        return false;
    out = _channels[(mask & 1U) != 0U ? 0U : 1U].frame; // 两侧同时有帧时优先左
    return true;
}

bool IrReceiverClass::read(IrSide side, IrFrame &out)
{
    const size_t idx = side_index(side);
    const uint32_t bit = 1U << idx;
    const uint32_t mask = _pending_mask.fetch_and(~bit, std::memory_order_acq_rel);
    if ((mask & bit) == 0U)
        return false;
    out = _channels[idx].frame;
    return true;
}

uint32_t IrReceiverClass::last_rx_ms(IrSide side) const
{
    return _channels[side_index(side)].last_activity_ms;
}

bool IrReceiverClass::signal_seen(IrSide side, uint32_t within_ms) const
{
    const uint32_t last = _channels[side_index(side)].last_activity_ms;
    // uint32_t 回绕时减法仍成立，无需额外处理
    return last != 0U && (now_ms() - last) <= within_ms;
}
