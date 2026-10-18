#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

extern "C" {
#include "main.h"
#include "usart.h"
}

sp::DBus remote(&huart3);

extern "C" void uart_task(void const *argument)
{
  remote.request();

  while (true) {
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}