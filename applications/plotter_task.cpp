#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 引用底盘任务里的全局数组
extern volatile float g_plot_data[8];
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    // 发送 8 个通道
    // 1,3,5,7 是目标速度，2,4,6,8 是实际速度
    plotter.plot(
      g_plot_data[0], g_plot_data[1], // FL 目标, FL 实际
      g_plot_data[2], g_plot_data[3], // FR 目标, FR 实际
      g_plot_data[4], g_plot_data[5], // RL 目标, RL 实际
      g_plot_data[6], g_plot_data[7]  // RR 目标, RR 实际
    );
    osDelay(10);
  }
}