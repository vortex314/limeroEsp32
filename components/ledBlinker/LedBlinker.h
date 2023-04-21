#ifndef LEDBLINKER_H
#define LEDBLINKER_H

#include <limero.h>

#include "driver/gpio.h"
#include "rom/gpio.h"

class LedBlinker : public Actor {
  uint32_t _pin;
  int _on = 0;
  ValueFlow<bool> _blinkSlow;
  TimerSource& _blinkTimer;

 public:
  Sink<bool>& blinkSlow() { return _blinkSlow; };
  LedBlinker(Thread& thr, uint32_t pin, uint32_t delay);
  void init();
  void delay(uint32_t d);
  void onNext(const TimerSource&);
};

#endif  // LEDBLINKER_H
