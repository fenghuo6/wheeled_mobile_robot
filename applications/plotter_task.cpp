#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

extern volatile float g_plot_data[8];
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    // 发送 8 个通道，直接看四个轮子的跟随情况
    plotter.plot(
      g_plot_data[0], g_plot_data[1], // FL 目标, FL 实际
      g_plot_data[2], g_plot_data[3], // FR 目标, FR 实际
      g_plot_data[4], g_plot_data[5], // RL 目标, RL 实际
      g_plot_data[6], g_plot_data[7]  // RR 目标, RR 实际
    );
    osDelay(10); // 100Hz
  }
}