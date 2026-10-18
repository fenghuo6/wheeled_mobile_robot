#include "cmsis_os.h"
#include <cmath>

extern "C"
{
#include "main.h"
#include "can.h"
}

#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "referee/pm02/pm02.hpp"

extern sp::DBus remote;
extern sp::PM02 pm02;

// ========== CAN与电机对象 ==========
sp::CAN can1(&hcan1);
sp::RM_Motor motor_fr(1, sp::RM_Motors::M3508, 13.7f); // 右前
sp::RM_Motor motor_fl(2, sp::RM_Motors::M3508, 13.7f); // 左前
sp::RM_Motor motor_rl(3, sp::RM_Motors::M3508, 13.7f); // 左后
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, 13.7f); // 右后

// ========== 速度环 PID ==========
sp::PID pid_fl(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);
sp::PID pid_fr(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);
sp::PID pid_rl(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);
sp::PID pid_rr(0.001f, 1.0f, 0.01f, 0.0f, 16384.0f, 2000.0f, 0.5f, false, true);

// ========== 功率模型参数 ==========
constexpr float K1 = 2.7f;   // 你测出的最佳峰值拟合
constexpr float K2 = 0.015f; // 旋转拟合
constexpr float K3 = 3.5f;   // 静态功耗

// ========== 全局共享数据 ==========
volatile float g_actual_power = 0.0f;    
volatile float g_predicted_power = 0.0f; 
volatile float g_power_limit = 80.0f;    
volatile float g_plot_data[8] = {0};     

// ========== 方向与运动学参数 ==========
constexpr float kDirFL = 1.0f;
constexpr float kDirRL = 1.0f;
constexpr float kDirFR = -1.0f;
constexpr float kDirRR = -1.0f;

constexpr float kWheelRadius = 0.077f;
constexpr float kLxLy = 0.185f + 0.165f; // 0.35m
constexpr float kMaxWheelSpeed = 3.0f;
constexpr float kMaxOmega = 3.0f;
constexpr float kMaxCurrent = 5.0f;      // 实际输出力矩限幅 (N·m)
constexpr float kRampStep = 0.05f;

namespace
{
    float vx_target = 0.0f;
    float vy_target = 0.0f;
    float omega_target = 0.0f;
}

// ========== 底盘任务 ==========
extern "C" void chassis_task(void const *argument)
{
    can1.config();
    can1.start();

    while (true)
    {
        // 读取裁判系统功率上限
        g_power_limit = pm02.robot_status.chassis_power_limit;
        if (g_power_limit < 10.0f || g_power_limit > 200.0f) g_power_limit = 80.0f;

        // 1. 下档失能保护
        if (remote.sw_r == sp::DBusSwitchMode::DOWN)
        {
            motor_fl.cmd(0.0f); motor_fr.cmd(0.0f); motor_rl.cmd(0.0f); motor_rr.cmd(0.0f);
            pid_fl.clear(); pid_fr.clear(); pid_rl.clear(); pid_rr.clear();
            vx_target = 0; vy_target = 0; omega_target = 0;

            motor_fl.write(can1.tx_data); motor_fr.write(can1.tx_data);
            motor_rl.write(can1.tx_data); motor_rr.write(can1.tx_data);
            can1.send(0x200);
            osDelay(1);
            continue;
        }

        // 2. 遥控器断线保护
        if (!remote.is_alive(HAL_GetTick()))
        {
            motor_fl.cmd(0.0f); motor_fr.cmd(0.0f); motor_rl.cmd(0.0f); motor_rr.cmd(0.0f);
            pid_fl.clear(); pid_fr.clear(); pid_rl.clear(); pid_rr.clear();
            vx_target = 0; vy_target = 0; omega_target = 0;

            motor_fl.write(can1.tx_data); motor_fr.write(can1.tx_data);
            motor_rl.write(can1.tx_data); motor_rr.write(can1.tx_data);
            can1.send(0x200);
            osDelay(1);
            continue;
        }

        // 3. 上档：单电机调试模式
        if (remote.sw_r == sp::DBusSwitchMode::UP)
        {
            static int test_motor = 0;
            static uint32_t last_switch_ms = 0;
            if (HAL_GetTick() - last_switch_ms > 300)
            {
                if (remote.ch_lh > 0.5f) { test_motor = (test_motor + 1) % 4; last_switch_ms = HAL_GetTick(); }
                if (remote.ch_lh < -0.5f) { test_motor = (test_motor + 3) % 4; last_switch_ms = HAL_GetTick(); }
            }
            float current = remote.ch_lv * 0.5f;
            motor_fl.cmd(test_motor == 0 ? current : 0.0f);
            motor_fr.cmd(test_motor == 1 ? current : 0.0f);
            motor_rl.cmd(test_motor == 2 ? current : 0.0f);
            motor_rr.cmd(test_motor == 3 ? current : 0.0f);
        }
        // 4. 中档：正常运动模式
        else if (remote.sw_r == sp::DBusSwitchMode::MID)
        {
            // 摇杆映射
            float vx_raw = remote.ch_lv * kMaxWheelSpeed;
            float vy_raw = -remote.ch_lh * kMaxWheelSpeed;
            float omega_raw = remote.ch_lu * kMaxOmega;

            // 目标速度斜坡
            vx_target += sp::limit_max(vx_raw - vx_target, kRampStep);
            vy_target += sp::limit_max(vy_raw - vy_target, kRampStep);
            omega_target += sp::limit_max(omega_raw - omega_target, kRampStep);

            // 麦轮运动学逆解
            float v_fl = vx_target - vy_target - omega_target * kLxLy;
            float v_fr = vx_target + vy_target + omega_target * kLxLy;
            float v_rl = vx_target + vy_target - omega_target * kLxLy;
            float v_rr = vx_target - vy_target + omega_target * kLxLy;

            float w_fl = v_fl / kWheelRadius * kDirFL;
            float w_fr = v_fr / kWheelRadius * kDirFR;
            float w_rl = v_rl / kWheelRadius * kDirRL;
            float w_rr = v_rr / kWheelRadius * kDirRR;

            // 速度环计算
            pid_fl.calc(w_fl, motor_fl.speed);
            pid_fr.calc(w_fr, motor_fr.speed);
            pid_rl.calc(w_rl, motor_rl.speed);
            pid_rr.calc(w_rr, motor_rr.speed);

            // ================== 功率预测（无滤波） ==================
            sp::RM_Motor *motors[4] = {&motor_fl, &motor_fr, &motor_rl, &motor_rr};
            float sum_tau_omega = 0, sum_tau_sq = 0, sum_omega_sq = 0;

            for (int i = 0; i < 4; i++)
            {
                float tau = motors[i]->torque; // 实际反馈扭矩
                float omega = motors[i]->speed;
                
                sum_tau_omega += tau * omega; // 无绝对值
                sum_tau_sq += tau * tau;
                sum_omega_sq += omega * omega;
            }

            float raw_predicted_power = sum_tau_omega + K1 * sum_tau_sq + K2 * sum_omega_sq + K3;
            if (raw_predicted_power < 0.0f) raw_predicted_power = 0.0f;

            g_predicted_power = raw_predicted_power; // 不使用滤波

            // ================== 功率限制核心逻辑 ==================
            float final_cmd[4] = {pid_fl.out, pid_fr.out, pid_rl.out, pid_rr.out};
            
            // 提前 5W 开始限制，完美吃掉刹车时的模型误差
            float control_power_limit = g_power_limit - 5.0f;

            // 1. 根据预测功率缩放电流
            if (g_predicted_power > control_power_limit && sum_tau_sq > 0.01f)
            {
                float a = K1 * sum_tau_sq;
                float b = sum_tau_omega;
                float c = K2 * sum_omega_sq + K3 - control_power_limit;
                float delta = b * b - 4 * a * c;
                float scale_K = 1.0f;

                if (delta >= 0) {
                    scale_K = (-b + std::sqrt(delta)) / (2 * a);
                    if (scale_K < 0) scale_K = 0;
                    if (scale_K > 1) scale_K = 1;
                } else { 
                    scale_K = 0; 
                }

                for (int i = 0; i < 4; i++) {
                    final_cmd[i] *= scale_K;
                }
            }

            // 2. 实际功率兜底硬砍（如果功率计反馈快爆表了，不管预测直接砍 10%）
            if (g_actual_power > (g_power_limit - 2.0f) && sum_tau_sq > 0.01f) {
                for (int i = 0; i < 4; i++) final_cmd[i] *= 0.9f;
            }

            // 发送最终电流（限幅）
            motor_fl.cmd(sp::limit_max(final_cmd[0], kMaxCurrent));
            motor_fr.cmd(sp::limit_max(final_cmd[1], kMaxCurrent));
            motor_rl.cmd(sp::limit_max(final_cmd[2], kMaxCurrent));
            motor_rr.cmd(sp::limit_max(final_cmd[3], kMaxCurrent));

            // 更新波形
            g_plot_data[0] = w_fl; g_plot_data[1] = motor_fl.speed;
            g_plot_data[2] = w_fr; g_plot_data[3] = motor_fr.speed;
            g_plot_data[4] = g_predicted_power;
            g_plot_data[5] = g_actual_power;
            g_plot_data[6] = g_power_limit;
        }

        // 底层 CAN 发送
        motor_fl.write(can1.tx_data); motor_fr.write(can1.tx_data);
        motor_rl.write(can1.tx_data); motor_rr.write(can1.tx_data);
        can1.send(0x200);
        osDelay(1);
    }
}

// ========== CAN 接收回调 ==========
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto stamp_ms = osKernelSysTick();
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (hcan == &hcan1)
        {
            can1.recv();
            if (can1.rx_id == motor_fl.rx_id) motor_fl.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_fr.rx_id) motor_fr.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_rl.rx_id) motor_rl.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_rr.rx_id) motor_rr.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == 0x301)
            {
                int16_t power_raw = (can1.rx_data[4] << 8) | can1.rx_data[5];
                g_actual_power = power_raw / 100.0f;
            }
        }
    }
}