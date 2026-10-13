#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::Mahony imu;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    plotter.plot(imu.yaw, imu.pitch, imu.roll);
    osDelay(10);
  }
}