#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 引用底盘任务里的全局数组
extern volatile float g_plot_data[8];

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    // 裁判要求：通道1=预测功率，通道2=实际功率
    plotter.plot(
      g_plot_data[6], // 通道1: 预测(目标)功率 (W)
      g_plot_data[5]  // 通道2: 实际功率 (W)
    );
    osDelay(10); // 100Hz 发送
  }
}