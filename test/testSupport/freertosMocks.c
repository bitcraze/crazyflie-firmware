#include <stdint.h>

uint32_t xTaskGetTickCount()
{
  return 1;
}

void vTaskDelay(uint32_t delay) {
  return;
}