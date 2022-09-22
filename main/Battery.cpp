#ifdef MAIN_BATTERY
#include <Hardware.h>
#include <INA3221.h>
#include <LedBlinker.h>
#include <Log.h>
#include <RedisSpineCbor.h>
#include <Udp.h>
#include <Wifi.h>
#include <limero.h>

Log logger;
Thread workerThread("worker");

Uext uext(1);
I2C& i2c = uext.getI2C();
INA3221 ina3221(i2c, INA3221_ADDR40_GND);
LedBlinker ledBlinker(workerThread, GPIO_NUM_2, 100);

Wifi wifi(workerThread);
Udp udp(workerThread);
RedisSpineCbor redis(workerThread, "battery");
TimerSource timerSource(workerThread, 100, true, "clock");
TimerSource reportTimer(workerThread, 1000, true, "reportTimer");

extern "C" void app_main() {
  ledBlinker.init();
  wifi.init();

  redis.txdCbor >> udp.txd();
  udp.rxd() >> redis.rxdCbor;

  wifi.connected >> udp.wifiConnected();

  redis.connected >> ledBlinker.blinkSlow();

  udp.port(9000);
  udp.dst("192.168.0.197:9001");
  redis.init();

  auto& test = redis.publisher<float>("tester/voltage1");

  i2c.setClock(100000);
  i2c.init();
  i2c.onFailure(
      [](Error& error) { DEBUG("%s:%d", error.message, error.code); });

  INFO(" scanning I2C bus ...");
  uint8_t i2cAddress = 0;
  uint8_t count = 0;
  uint8_t bytes[] = {0x00};
  reportTimer >> [&](const TimerMsg&) { test.on(3.14); };
  timerSource >> [&](const TimerMsg&) {
    INFO(" INA3221 Manufacture Id 0x%X", ina3221.getManufID());
    INFO(" INA3221         Die Id 0x%X", ina3221.getDieID());
    test.on(ina3221.getShuntVoltage(INA3221_CH1));
  };
  workerThread.start();
}
#endif