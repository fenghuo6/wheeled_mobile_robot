#include "cmsis_os.h"
#include <cmath>

extern "C" {
#include "main.h"
#include "can.h"
}

#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "tools/math_tools/math_tools.hpp"

// 对外暴露的全局变量
sp::CAN can1(&hcan1);
sp::RM_Motor motor_fr(1, sp::RM_Motors::M3508, 13.7f);
sp::RM_Motor motor_fl(2, sp::RM_Motors::M3508, 13.7f);
sp::RM_Motor motor_rl(3, sp::RM_Motors::M3508, 13.7f);
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, 13.7f);
extern sp::DBus remote;

// 给 plotter_task 用的调试数据
volatile float g_plot_data[8] = {0};

// PID
sp::PID pid_fl(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);
sp::PID pid_fr(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);
sp::PID pid_rl(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);
sp::PID pid_rr(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);

// 方向修正（明天实测如果反了改符号）
constexpr float kDirFL = 1.0f;
constexpr float kDirRL = 1.0f;
constexpr float kDirFR = -1.0f;
constexpr float kDirRR = -1.0f;

// 运动学
constexpr float kWheelRadius = 0.077f;
constexpr float kLxLy = 0.185f + 0.165f; // 0.35m

// 安全限制
constexpr float kMaxWheelSpeed = 3.0f;
constexpr float kMaxOmega = 3.0f;
constexpr float kMaxCurrent = 5.0f;
constexpr float kRampStep = 0.05f;

namespace {
float vx_target = 0.0f;
float vy_target = 0.0f;
float omega_target = 0.0f;
}

extern "C" void chassis_task(void const *argument)
{
    // 在这里初始化CAN，避免竞争
    can1.config();
    can1.start();

    while (true) {
        // 第一层：下档失能
        if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
            motor_fl.cmd(0.0f); motor_fr.cmd(0.0f); motor_rl.cmd(0.0f); motor_rr.cmd(0.0f);
            pid_fl.clear(); pid_fr.clear(); pid_rl.clear(); pid_rr.clear();
            vx_target = 0; vy_target = 0; omega_target = 0;
            
            motor_fl.write(can1.tx_data); motor_fr.write(can1.tx_data);
            motor_rl.write(can1.tx_data); motor_rr.write(can1.tx_data);
            can1.send(0x200);
            osDelay(1);
            continue;
        }

        // 第二层：离线保护
        if (!remote.is_alive(HAL_GetTick())) {
            motor_fl.cmd(0.0f); motor_fr.cmd(0.0f); motor_rl.cmd(0.0f); motor_rr.cmd(0.0f);
            pid_fl.clear(); pid_fr.clear(); pid_rl.clear(); pid_rr.clear();
            vx_target = 0; vy_target = 0; omega_target = 0;
            
            motor_fl.write(can1.tx_data); motor_fr.write(can1.tx_data);
            motor_rl.write(can1.tx_data); motor_rr.write(can1.tx_data);
            can1.send(0x200);
            osDelay(1);
            continue;
        }

        // 调试模式（上档）
        if (remote.sw_r == sp::DBusSwitchMode::UP) {
            static int test_motor = 0;
            static uint32_t last_switch_ms = 0;

            if (HAL_GetTick() - last_switch_ms > 300) {
                if (remote.ch_lh > 0.5f) { test_motor = (test_motor + 1) % 4; last_switch_ms = HAL_GetTick(); }
                if (remote.ch_lh < -0.5f) { test_motor = (test_motor + 3) % 4; last_switch_ms = HAL_GetTick(); }
            }

            float current = remote.ch_lv * 0.5f;

            motor_fl.cmd(test_motor == 0 ? current : 0.0f);
            motor_fr.cmd(test_motor == 1 ? current : 0.0f);
            motor_rl.cmd(test_motor == 2 ? current : 0.0f);
            motor_rr.cmd(test_motor == 3 ? current : 0.0f);

            g_plot_data[0] = static_cast<float>(test_motor);
            g_plot_data[1] = motor_fl.speed;
            g_plot_data[2] = motor_fr.speed;
            g_plot_data[3] = motor_rl.speed;
            g_plot_data[4] = motor_rr.speed;
        }
        // 正常模式（中档）
        else if (remote.sw_r == sp::DBusSwitchMode::MID) {
            float vx_raw = remote.ch_lv * kMaxWheelSpeed;
            float vy_raw = -remote.ch_lh * kMaxWheelSpeed;
            float omega_raw = remote.ch_lu * kMaxOmega;

            vx_target += sp::limit_max(vx_raw - vx_target, kRampStep);
            vy_target += sp::limit_max(vy_raw - vy_target, kRampStep);
            omega_target += sp::limit_max(omega_raw - omega_target, kRampStep);

            float v_fl = vx_target - vy_target - omega_target * kLxLy;
            float v_fr = vx_target + vy_target + omega_target * kLxLy;
            float v_rl = vx_target + vy_target - omega_target * kLxLy;
            float v_rr = vx_target - vy_target + omega_target * kLxLy;

            float w_fl = v_fl / kWheelRadius * kDirFL;
            float w_fr = v_fr / kWheelRadius * kDirFR;
            float w_rl = v_rl / kWheelRadius * kDirRL;
            float w_rr = v_rr / kWheelRadius * kDirRR;

            pid_fl.calc(w_fl, motor_fl.speed);
            pid_fr.calc(w_fr, motor_fr.speed);
            pid_rl.calc(w_rl, motor_rl.speed);
            pid_rr.calc(w_rr, motor_rr.speed);

            motor_fl.cmd(sp::limit_max(pid_fl.out, kMaxCurrent));
            motor_fr.cmd(sp::limit_max(pid_fr.out, kMaxCurrent));
            motor_rl.cmd(sp::limit_max(pid_rl.out, kMaxCurrent));
            motor_rr.cmd(sp::limit_max(pid_rr.out, kMaxCurrent));

            g_plot_data[0] = w_fl; g_plot_data[1] = motor_fl.speed;
            g_plot_data[2] = w_fr; g_plot_data[3] = motor_fr.speed;
            g_plot_data[4] = w_rl; g_plot_data[5] = motor_rl.speed;
            g_plot_data[6] = w_rr; g_plot_data[7] = motor_rr.speed;
        }

        motor_fl.write(can1.tx_data); motor_fr.write(can1.tx_data);
        motor_rl.write(can1.tx_data); motor_rr.write(can1.tx_data);
        can1.send(0x200);
        osDelay(1);
    }
}

// CAN 接收回调（唯一的）
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto stamp_ms = osKernelSysTick();
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (hcan == &hcan1) {
            can1.recv();
            if (can1.rx_id == motor_fl.rx_id) motor_fl.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_fr.rx_id) motor_fr.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_rl.rx_id) motor_rl.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_rr.rx_id) motor_rr.read(can1.rx_data, stamp_ms);
        }
    }
}