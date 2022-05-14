#ifdef MAIN_AS5600_TEST
#include <As5600.h>
#include <LedBlinker.h>
#include <Hardware.h>
#include <Log.h>
#include <limero.h>
Log logger;
extern "C" void app_main() {
  Thread workerThread("worker");
  Uext uext(1);
  As5600 as5600(uext);
  LedBlinker ledBlinker(workerThread, GPIO_NUM_2, 100);
  ledBlinker.init();
  as5600.init();
  as5600.onFailure(
      [](Error& error) { ERROR("%s:%d", error.message, error.code); });

  TimerSource timerSource(workerThread, 1000, true, "clock");

  timerSource >> [&](const TimerMsg&) {
    int angle = as5600.angle();
    INFO("position  : %d", angle);
  };
  workerThread.run();
}
#endif