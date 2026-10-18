#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

extern volatile float g_plot_data[8];
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    // 发送 7 个通道
    plotter.plot(
      g_plot_data[0], g_plot_data[1], // 1,2: FL 目标速度 vs 实际速度
      g_plot_data[2], g_plot_data[3], // 3,4: FR 目标速度 vs 实际速度
      g_plot_data[4], g_plot_data[5], // 5,6: 预测功率 vs 实际功率
      g_plot_data[6]                  // 7: 裁判系统下发的功率上限
    );
    osDelay(10); 
  }
}