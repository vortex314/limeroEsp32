#ifdef MAIN_I2C_SCANNER
#include <Hardware.h>
#include <LedBlinker.h>
#include <Log.h>
#include <limero.h>
Log logger;
extern "C" void app_main() {
  Thread workerThread("worker");
  Uext uext(1);
  LedBlinker ledBlinker(workerThread, GPIO_NUM_2, 100);
  ledBlinker.init();
  I2C& i2c = uext.getI2C();
  i2c.setClock(100000);
  i2c.init();
  i2c.onFailure(
      [](Error& error) { DEBUG("%s:%d", error.message, error.code); });

  TimerSource timerSource(workerThread, 100, true, "clock");
  INFO(" scanning I2C bus ...");
  uint8_t i2cAddress = 0;
  uint8_t count = 0;
  uint8_t bytes[] = {0x00};
  timerSource >> [&](const TimerMsg&) {
    i2c.setSlaveAddress(i2cAddress);
    int rc = i2c.write(bytes, 1);
    if (rc == 0) {
      INFO("found device at 0x%.2X : %d", i2cAddress, rc);
      count++;
    }
    i2cAddress++;
    if (i2cAddress > 0x7F) {
      INFO("found devices : %d ", count);
      i2cAddress = 0;
      count = 0;
    }
  };

  workerThread.run();
}
#endif