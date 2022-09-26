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
std::vector<std::string> modeNames = {"idle", "charging", "decharging"};
ValueFlow<Mode> mode(M_IDLE);

extern "C" void app_main() {
  loadPin.setMode(DigitalOut::Mode::DOUT_PULL_UP);
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

  mode >> *new SinkFunction<Mode>([&](const Mode& m) {
    INFO("mode change %s", modeNames[m].c_str());
    switch (m) {
      case M_IDLE:
        loadPin.write(1);
        chargePin.write(1);
        break;
      case M_CHARGING:
        loadPin.write(1);
        chargePin.write(0);
        break;
      case M_DECHARGING:
        loadPin.write(0);
        chargePin.write(1);
        break;
    }
  });

  redis.subscriber<std::string>("tester/mode") >>
      *new SinkFunction<std::string>([&](const std::string& stringMode) {
        INFO("subscribe mode: %s", stringMode.c_str());
        for (int i = 0; i < modeNames.size(); i++) {
          if (stringMode == modeNames[i]) {
            mode = (Mode)i;
            return;
          }
        }
      });

  redis.subscriber<bool>("tester/reset") >>
      *new SinkFunction<bool>([&](const bool&) {
        INFO("reset");
        ch1Power = 0;
        ch2Power = 0;
        ch3Power = 0;
      });

  mode = M_IDLE;

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
  ina3221.setAveragingMode(INA3221_REG_CONF_AVG_128);

  ch1Time = Sys::millis();
  ch2Time = Sys::millis();
  ch3Time = Sys::millis();

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
    float batteryVoltage =
        ina3221.getVoltage(INA3221_CH2) + ina3221.getCurrent(INA3221_CH2) * 0.1;
    if (redis.connected()) {
      props.start()
          .write('[')
          .write("pub")
          .write("src/battery/tester/")
          .write('{');

      props.write("current_1").write(ina3221.getCurrent(INA3221_CH1));
      props.write("current_2").write(ina3221.getCurrent(INA3221_CH2));
      props.write("current_3").write(ina3221.getCurrent(INA3221_CH3));

      props.write("voltage_1").write(ina3221.getVoltage(INA3221_CH1));
      props.write("voltage_2").write(batteryVoltage);
      props.write("voltage_3").write(ina3221.getVoltage(INA3221_CH3));

      props.write("power_1").write(ch1Power / 1000);
      props.write("power_2").write(ch2Power / 1000);
      props.write("power_3").write(ch3Power / 1000);

      props.write("mode").write(modeNames[mode()]);

      props.write('}').write(']').end();
      redis.txdCbor.on(props);
    }
    // decharged enough
    if (batteryVoltage < 3.0) {
      INFO("battery voltage too low %f ", batteryVoltage);
      mode.on(M_IDLE);
    }
  };
  workerThread.start();
}
#endif