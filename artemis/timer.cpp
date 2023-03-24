#include "timer.h"

//*****UN USED *****

namespace rtos {

  osStatus_t Timer::start(osTimerFunc_t callback, void* arg, uint32_t dur) {
    config.cb_mem = &controlBlock;
    config.cb_size = sizeof(controlBlock);

    id = osTimerNew(callback, osTimerPeriodic, arg, &config);
    return osTimerStart(id, dur);
  }

  osStatus_t Timer::stop() {
    return osTimerStop(id);
  }

}
