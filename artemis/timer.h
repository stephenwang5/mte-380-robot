#ifndef TIMER_H
#define TIMER_H

namespace rtos {
  class Timer {
    osRtxTimer_t controlBlock;
    osTimerAttr_t config;
  public:
    osTimerId id;

    Timer() = default;
    osStatus_t start(osTimerFunc_t callback, void* arg, uint32_t dur);
    osStatus_t stop();
  };
}

#endif // TIMER_H
