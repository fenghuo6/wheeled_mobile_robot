#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

extern volatile float g_plot_data[8];
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    // 先发前5个通道（调试模式足够看）
    plotter.plot(
      g_plot_data[0], g_plot_data[1], g_plot_data[2], 
      g_plot_data[3], g_plot_data[4]);
    osDelay(10);
  }
}