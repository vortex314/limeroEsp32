#ifdef MAIN_BATTERY
#include <Hardware.h>
#include <LedBlinker.h>
#include <Log.h>
#include <RedisSpineCbor.h>
#include <Udp.h>
#include <Wifi.h>
#include <limero.h>

Log logger;
extern "C" void app_main() {
  Thread workerThread("worker");
  Thread udpThread("udp");

  Uext uext(1);
  I2C& i2c = uext.getI2C();
  LedBlinker ledBlinker(workerThread, GPIO_NUM_2, 100);

  Wifi wifi(workerThread);
  Udp udp(udpThread);
  UdpMsg udpMsg;
  RedisSpineCbor redis(workerThread, "battery");

  ledBlinker.init();
  wifi.init();
  udp.port(9000);
  udp.init();
  redis.init();

  redis.txdFrame >> udp.txd();
  udp.rxd() >> redis.rxdFrame;

  // udp.dst("192.168.0.197");
  UdpAddress::fromUri(udpMsg.dst, "192.168.0.197:9001");

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
    udp.send(udpMsg);
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