#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

volatile float g_motor_a_angle = 0.0f;
volatile float g_motor_b_angle = 0.0f;
volatile float g_motor_a_speed = 0.0f;
volatile float g_motor_b_speed = 0.0f;

extern "C" void can_task()
{
  can1.config();
  can1.start();

  while (true) {
    // 电流由 gimbal_task 通过 motor_a.cmd() 设置
    motor_a.write(can1.tx_data);
    motor_b.write(can1.tx_data);
    can1.send(motor_a.tx_id);

    g_motor_a_angle = motor_a.angle;
    g_motor_b_angle = motor_b.angle;
    g_motor_a_speed = motor_a.speed;
    g_motor_b_speed = motor_b.speed;

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}