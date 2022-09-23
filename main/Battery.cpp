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
DigitalOut& loadPin = uext.getDigitalOut(LP_TXD);
DigitalOut& chargePin = uext.getDigitalOut(LP_MOSI);
LedBlinker ledBlinker(workerThread, GPIO_NUM_2, 100);

Wifi wifi(workerThread);
Udp udp(workerThread);
RedisSpineCbor redis(workerThread, "battery");
TimerSource timerSource(workerThread, 1000, true, "clock");
CborEncoder props(1000);

double ch1Power, ch2Power, ch3Power;
uint64_t ch1Time, ch2Time, ch3Time;

typedef enum { M_IDLE, M_CHARGING, M_DECHARGING } Mode;

Mode mode;

extern "C" void app_main() {
  loadPin.init();
  chargePin.init();
  ledBlinker.init();
  wifi.init();

  redis.txdCbor >> udp.txd();
  udp.rxd() >> redis.rxdCbor;
  wifi.connected >> udp.wifiConnected();
  redis.connected >> ledBlinker.blinkSlow();

  udp.port(9000);
  udp.dst("192.168.0.197:9001");
  redis.init();
  mode = M_IDLE;

  redis.subscriber<std::string>("tester/mode") >> [&](const std::string& m) {
    if (m == "idle") {
      chargePin.write(0);
      loadPin.write(0);
    } else if (m == "charge") {
      chargePin.write(1);
      loadPin.write(0);
    } else if (m == "load") {
      chargePin.write(0);
      loadPin.write(1);
    }
  };

    /*
      1000 mAh 4.2 V ==> 4.2 Wh = 15120 Ws = 15120 Joule
      1 kWh = 3600*1000 Ws = 3.6 MJ
    */

    i2c.setClock(100000);
    i2c.init();
    i2c.onFailure(
        [](Error& error) { DEBUG("%s:%d", error.message, error.code); });
    ina3221.setBusConversionTime(INA3221_REG_CONF_CT_1100US);
    ina3221.setShuntRes(100, 100, 100);
    ina3221.setAveragingMode(INA3221_REG_CONF_AVG_512);

    ch1Time = Sys::millis();
    ch2Time = Sys::millis();
    ch3Time = Sys::millis();

    INFO(" scanning I2C bus ...");
    timerSource >> [&](const TimerMsg&) {
      INFO(" INA3221 Manufacture Id 0x%X", ina3221.getManufID());
      INFO(" INA3221         Die Id 0x%X", ina3221.getDieID());
      ch1Power += ina3221.getCurrent(INA3221_CH1) *
                  ina3221.getVoltage(INA3221_CH1) * (Sys::millis() - ch1Time);
      ch2Power += ina3221.getCurrent(INA3221_CH2) *
                  ina3221.getVoltage(INA3221_CH2) * (Sys::millis() - ch2Time);
      ch3Power += ina3221.getCurrent(INA3221_CH3) *
                  ina3221.getVoltage(INA3221_CH3) * (Sys::millis() - ch3Time);
      ch1Time = Sys::millis();
      ch2Time = Sys::millis();
      ch3Time = Sys::millis();
      props.start().write('[').write("pub").write("src/battery/tester/").write('{');

      props.write("current_1").write(ina3221.getCurrent(INA3221_CH1));
      props.write("current_2").write(ina3221.getCurrent(INA3221_CH2));
      props.write("current_3").write(ina3221.getCurrent(INA3221_CH3));

      props.write("voltage_1").write(ina3221.getVoltage(INA3221_CH1));
      props.write("voltage_2").write(ina3221.getVoltage(INA3221_CH2));
      props.write("voltage_3").write(ina3221.getVoltage(INA3221_CH3));

      props.write("power_1").write(ch1Power / 1000);
      props.write("power_2").write(ch2Power / 1000);
      props.write("power_3").write(ch3Power / 1000);

      props.write('}').write(']').end();
      redis.txdCbor.on(props);
      // decharged enough
      if (ina3221.getCurrent(INA3221_CH1) < 3.7) loadPin.write(0);
    };
    workerThread.start();
  }
#endif