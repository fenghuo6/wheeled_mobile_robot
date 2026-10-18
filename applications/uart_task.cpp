#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"

extern "C" {
#include "main.h"
#include "usart.h"
}

// 实例化遥控器和裁判系统 (假设裁判系统接在 UART6)
sp::DBus remote(&huart3);
sp::PM02 pm02(&huart6); 

extern "C" void uart_task(void const *argument)
{
  remote.request();
  pm02.request();

  while (true) {
    // 维护 PM02 的超时
    taskENTER_CRITICAL();
    pm02.update_radar_data_timeout(HAL_GetTick());
    taskEXIT_CRITICAL();
    osDelay(10);
  }
}

// 唯一的接收回调
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();
  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
  else if (huart == pm02.huart) { // 判断是不是 UART6
    pm02.update(Size);
    pm02.request();
  }
}

// 唯一的错误回调
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  else if (huart == pm02.huart) {
    pm02.request();
  }
}