#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

///
/// 板载 LED（GPIO46，高电平亮、低电平灭）
///
/// 闪烁由独立 FreeRTOS 任务驱动；led_start_blink() 在任务已存在时只更新亮/灭时长，
/// 不会重复创建线程。实现全部放在头文件内，全局单例 Led 为 inline 变量，
/// 多个翻译单元包含本头文件时共享同一实例。
///
class LedClass
{
public:
    void led_on(void)
    {
        ensure_initialized();
        gpio_set_level(kLedPin, 1);
    }

    void led_off(void)
    {
        ensure_initialized();
        gpio_set_level(kLedPin, 0);
    }

    void led_toggle(void)
    {
        ensure_initialized();
        gpio_set_level(kLedPin, gpio_get_level(kLedPin) ? 0 : 1);
    }

    /// on_delay / off_delay 单位 ms；闪烁任务已存在时只更新时长并返回 true
    bool led_start_blink(uint32_t on_delay, uint32_t off_delay)
    {
        ensure_initialized();
        // 0 会让 vTaskDelay(0) 退化成忙等，钳到 1ms
        _on_delay_ms.store(on_delay ? on_delay : kMinDelayMs, std::memory_order_relaxed);
        _off_delay_ms.store(off_delay ? off_delay : kMinDelayMs, std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(_blink_mutex);
        if (_blink_task != nullptr)
        {
            return true;
        }
        TaskHandle_t handle = nullptr;
        if (xTaskCreate(blink_task, "led_blink", kStackSize, this, kPriority, &handle) != pdPASS)
        {
            return false;
        }
        _blink_task = handle;
        return true;
    }

    /// 停止闪烁任务，LED 保持停止瞬间的电平
    void led_stop_blink(void)
    {
        std::lock_guard<std::mutex> lock(_blink_mutex);
        if (_blink_task == nullptr)
        {
            return;
        }
        const TaskHandle_t handle = _blink_task;
        _blink_task = nullptr;
        vTaskDelete(handle);
    }

private:
    static constexpr gpio_num_t kLedPin = GPIO_NUM_46;
    static constexpr uint32_t kStackSize = 2048;
    static constexpr UBaseType_t kPriority = 2;
    static constexpr uint32_t kMinDelayMs = 1;

    // 懒初始化：全局对象构造阶段不碰硬件，首次使用时配置为输出并灭灯
    void ensure_initialized(void)
    {
        std::call_once(_init_once, []() {
            gpio_set_direction(kLedPin, GPIO_MODE_OUTPUT);
            gpio_set_level(kLedPin, 0);
        });
    }

    static void blink_task(void *arg)
    {
        auto *self = static_cast<LedClass *>(arg);
        while (true)
        {
            gpio_set_level(kLedPin, 1);
            vTaskDelay(pdMS_TO_TICKS(self->_on_delay_ms.load(std::memory_order_relaxed)));
            gpio_set_level(kLedPin, 0);
            vTaskDelay(pdMS_TO_TICKS(self->_off_delay_ms.load(std::memory_order_relaxed)));
        }
    }

    std::once_flag _init_once;
    std::atomic<uint32_t> _on_delay_ms{100};
    std::atomic<uint32_t> _off_delay_ms{900};
    std::mutex _blink_mutex;
    TaskHandle_t _blink_task = nullptr; // 受 _blink_mutex 保护
};

inline LedClass Led;
