#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 引用底盘任务里的全局数组
extern volatile float g_plot_data[8];

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    // 发送 8 个通道，按顺序打包发送
    plotter.plot(
      g_plot_data[0], g_plot_data[1], // 1,2: 左前 目标 vs 实际
      g_plot_data[2], g_plot_data[3], // 3,4: 右前 目标 vs 实际
      g_plot_data[4], g_plot_data[5], // 5,6: 预测功率 vs 实际功率
      g_plot_data[6], g_plot_data[7]  // 7,8: 限制线 vs 限制系数
    );
    osDelay(10); // 100Hz 发送
  }
}