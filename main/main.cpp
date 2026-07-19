#include "esplog.hpp"

extern "C" void app_main(void) {
  esplog::init(esplog::level::info, "ESP32_LOG");

  while (true) {
    esplog::info("Hello World!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
