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
sp::PID pid_fl(0.001f, 0.8f, 0.02f, 0.0f, 16384.0f, 2000.0f, 0.95f, false, false);
sp::PID pid_fr(0.001f, 0.8f, 0.02f, 0.0f, 16384.0f, 2000.0f, 0.95f, false, false);
sp::PID pid_rl(0.001f, 0.8f, 0.02f, 0.0f, 16384.0f, 2000.0f, 0.95f, false, false);
sp::PID pid_rr(0.001f, 0.8f, 0.02f, 0.0f, 16384.0f, 2000.0f, 0.95f, false, false);

// ========== 功率模型参数 ==========
constexpr float K1 = 2.7f;
constexpr float K2 = 0.015f;
constexpr float K3 = 3.5f;

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
constexpr float kLxLy = 0.185f + 0.165f;
constexpr float kMaxWheelSpeed = 3.0f;
constexpr float kMaxOmega = 3.0f;
constexpr float kMaxCurrent = 5.0f;
constexpr float kRampStep = 0.03f;

// ========== 自动模式相关 ==========
constexpr float kAutoTargetDistance = 3.0f;
constexpr float kAutoTargetSpeed = 1.0f;

namespace
{
    float vx_target = 0.0f;
    float vy_target = 0.0f;
    float omega_target = 0.0f;

    float auto_distance = 0.0f;
    bool auto_started = false;
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
        if (g_power_limit < 10.0f || g_power_limit > 200.0f)
            g_power_limit = 80.0f;

        // 1. 下档失能保护
        if (remote.sw_r == sp::DBusSwitchMode::DOWN)
        {
            auto_started = false;
            motor_fl.cmd(0.0f);
            motor_fr.cmd(0.0f);
            motor_rl.cmd(0.0f);
            motor_rr.cmd(0.0f);
            pid_fl.clear();
            pid_fr.clear();
            pid_rl.clear();
            pid_rr.clear();
            vx_target = 0;
            vy_target = 0;
            omega_target = 0;

            motor_fl.write(can1.tx_data);
            motor_fr.write(can1.tx_data);
            motor_rl.write(can1.tx_data);
            motor_rr.write(can1.tx_data);
            can1.send(0x200);
            osDelay(1);
            continue;
        }

        // 2. 遥控器断线保护
        if (!remote.is_alive(HAL_GetTick()))
        {
            auto_started = false;
            motor_fl.cmd(0.0f);
            motor_fr.cmd(0.0f);
            motor_rl.cmd(0.0f);
            motor_rr.cmd(0.0f);
            pid_fl.clear();
            pid_fr.clear();
            pid_rl.clear();
            pid_rr.clear();
            vx_target = 0;
            vy_target = 0;
            omega_target = 0;

            motor_fl.write(can1.tx_data);
            motor_fr.write(can1.tx_data);
            motor_rl.write(can1.tx_data);
            motor_rr.write(can1.tx_data);
            can1.send(0x200);
            osDelay(1);
            continue;
        }

        // ================== 计算目标速度 ==================
        float w_fl_target = 0.0f;
        float w_fr_target = 0.0f;
        float w_rl_target = 0.0f;
        float w_rr_target = 0.0f;

        // 3. 上档：自动直线行驶 3 米
        if (remote.sw_r == sp::DBusSwitchMode::UP)
        {
            if (!auto_started)
            {
                auto_started = true;
                auto_distance = 0.0f;
                pid_fl.clear();
                pid_fr.clear();
                pid_rl.clear();
                pid_rr.clear();
            }

            float target_v = (auto_distance >= kAutoTargetDistance) ? 0.0f : kAutoTargetSpeed;
            float target_w = target_v / kWheelRadius;

            w_fl_target = target_w * kDirFL;
            w_fr_target = target_w * kDirFR;
            w_rl_target = target_w * kDirRL;
            w_rr_target = target_w * kDirRR;

            float actual_w_avg = (motor_fl.speed * kDirFL + motor_fr.speed * kDirFR +
                                  motor_rl.speed * kDirRL + motor_rr.speed * kDirRR) /
                                 4.0f;
            auto_distance += actual_w_avg * kWheelRadius * 0.001f;
        }
        // 4. 中档：遥控器手动模式
        else if (remote.sw_r == sp::DBusSwitchMode::MID)
        {
            auto_started = false;

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

            w_fl_target = v_fl / kWheelRadius * kDirFL;
            w_fr_target = v_fr / kWheelRadius * kDirFR;
            w_rl_target = v_rl / kWheelRadius * kDirRL;
            w_rr_target = v_rr / kWheelRadius * kDirRR;
        }

        // ================== PID 计算 ==================
        constexpr float speed_alpha = 0.95f;
        static float fdb_fl = 0.0f, fdb_fr = 0.0f, fdb_rl = 0.0f, fdb_rr = 0.0f;

        fdb_fl = speed_alpha * motor_fl.speed + (1.0f - speed_alpha) * fdb_fl;
        fdb_fr = speed_alpha * motor_fr.speed + (1.0f - speed_alpha) * fdb_fr;
        fdb_rl = speed_alpha * motor_rl.speed + (1.0f - speed_alpha) * fdb_rl;
        fdb_rr = speed_alpha * motor_rr.speed + (1.0f - speed_alpha) * fdb_rr;

        pid_fl.calc(w_fl_target, fdb_fl);
        pid_fr.calc(w_fr_target, fdb_fr);
        pid_rl.calc(w_rl_target, fdb_rl);
        pid_rr.calc(w_rr_target, fdb_rr);

        // ================== 1. 提取目标扭矩（用于预测） ==================
        // 这里必须限幅，防止 PID 积分爆掉导致预测变成天文数字
        float tau_target[4] = {
            sp::limit_max(pid_fl.out, kMaxCurrent),
            sp::limit_max(pid_fr.out, kMaxCurrent),
            sp::limit_max(pid_rl.out, kMaxCurrent),
            sp::limit_max(pid_rr.out, kMaxCurrent)};

        // ================== 2. 功率预测（用目标扭矩，波形高高在上） ==================
        sp::RM_Motor *motors[4] = {&motor_fl, &motor_fr, &motor_rl, &motor_rr};
        float sum_tau_omega = 0, sum_tau_sq = 0, sum_omega_sq = 0;

        for (int i = 0; i < 4; i++)
        {
            float tau = tau_target[i];      // 用目标值算
            float omega = motors[i]->speed; // 用实际转速算

            sum_tau_omega += tau * omega;
            sum_tau_sq += tau * tau;
            sum_omega_sq += omega * omega;
        }

        float raw_predicted_power = sum_tau_omega + K1 * sum_tau_sq + K2 * sum_omega_sq + K3;
        if (raw_predicted_power < 0.0f)
            raw_predicted_power = 0.0f;
        g_predicted_power = raw_predicted_power;

        // ================== 3. 二次方程法限制 ==================
        float final_cmd[4] = {tau_target[0], tau_target[1], tau_target[2], tau_target[3]};

        // 提前 5W 开始限制
        float control_power_limit = g_power_limit - 5.0f;

        if (g_predicted_power > control_power_limit && sum_tau_sq > 0.01f)
        {
            float a = K1 * sum_tau_sq;
            float b = sum_tau_omega;
            float c = K2 * sum_omega_sq + K3 - control_power_limit;
            float delta = b * b - 4 * a * c;
            float scale_K = 1.0f;

            if (delta >= 0)
            {
                scale_K = (-b + std::sqrt(delta)) / (2 * a);
                if (scale_K < 0)
                    scale_K = 0;
                if (scale_K > 1)
                    scale_K = 1;
            }
            else
            {
                scale_K = 0;
            }

            for (int i = 0; i < 4; i++)
            {
                final_cmd[i] *= scale_K;
            }
        }

        // ================== 4. 实际功率闭环兜底（解决超限） ==================
        static float scale_K_actual = 1.0f;

        if (g_actual_power > g_power_limit)
        {
            scale_K_actual -= 0.02f; // 实测超标，猛砍
        }
        else if (g_actual_power < (control_power_limit - 5.0f))
        {
            scale_K_actual += 0.005f; // 实测低，慢慢恢复
        }

        if (scale_K_actual > 1.0f)
            scale_K_actual = 1.0f;
        if (scale_K_actual < 0.0f)
            scale_K_actual = 0.0f;

        // 叠加实际限制
        for (int i = 0; i < 4; i++)
        {
            final_cmd[i] *= scale_K_actual;
        }

        // 发送最终电流（限幅）
        motor_fl.cmd(sp::limit_max(final_cmd[0], kMaxCurrent));
        motor_fr.cmd(sp::limit_max(final_cmd[1], kMaxCurrent));
        motor_rl.cmd(sp::limit_max(final_cmd[2], kMaxCurrent));
        motor_rr.cmd(sp::limit_max(final_cmd[3], kMaxCurrent));

        // ================== 更新波形数据 ==================
        // 前4个通道：底盘PID跟随 (以左前、右前为例)
        g_plot_data[0] = w_fl_target;       // 通道1: 左前轮目标速度
        g_plot_data[1] = motor_fl.speed;    // 通道2: 左前轮实际速度
        g_plot_data[2] = w_fr_target;       // 通道3: 右前轮目标速度
        g_plot_data[3] = motor_fr.speed;    // 通道4: 右前轮实际速度
        
        // 后4个通道：功率控制与限制结果
        g_plot_data[4] = g_predicted_power; // 通道5: 预测功率
        g_plot_data[5] = g_actual_power;    // 通道6: 实际功率
        g_plot_data[6] = control_power_limit; // 通道7: 限制线（裁判上限 - 5W）
        
        // 通道8: 限制结果（也就是缩放系数，1.0代表未限制，越小代表砍得越多）
        // 注意：把限制代码里的 scale_K_actual 提出来赋值
        g_plot_data[7] = scale_K_actual;    // 通道8: 限制系数

        // 底层 CAN 发送
        motor_fl.write(can1.tx_data);
        motor_fr.write(can1.tx_data);
        motor_rl.write(can1.tx_data);
        motor_rr.write(can1.tx_data);
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
            if (can1.rx_id == motor_fl.rx_id)
                motor_fl.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_fr.rx_id)
                motor_fr.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_rl.rx_id)
                motor_rl.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == motor_rr.rx_id)
                motor_rr.read(can1.rx_data, stamp_ms);
            if (can1.rx_id == 0x301)
            {
                int16_t power_raw = (can1.rx_data[4] << 8) | can1.rx_data[5];
                g_actual_power = power_raw / 100.0f;
            }
        }
    }
}