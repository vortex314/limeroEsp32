
#include "Hardware.h"
#include "LedBlinker.h"
#include "esp_system.h"
#include "freertos/task.h"
#define STRINGIFY(X) #X
#define S(X) STRINGIFY(X)
//______________________________________________________________________
//

template <class T> class RequestFlow : public Flow<T, T> {
  Source<T> &_source;

public:
  RequestFlow(Source<T> &source) : _source(source) {}
  void request() { _source.request(); }
  void on(const T &t) { this->emit(t); }
};
//____________________________________________________________________________________
//
class Poller : public Actor {
  TimerSource _pollInterval;
  std::vector<Requestable *> _requestables;
  uint32_t _idx = 0;

public:
  ValueFlow<bool> connected;
  ValueFlow<uint32_t> interval = 500;
  Poller(Thread &t) : Actor(t), _pollInterval(t, 500, true) {
    _pollInterval >> [&](const TimerMsg tm) {
      if (_requestables.size() && connected())
        _requestables[_idx++ % _requestables.size()]->request();
    };
    interval >> [&](const uint32_t iv) { _pollInterval.interval(iv); };
  };

  /*
    template <class T> Source<T> &poll(Source<T> &source) {
      RequestFlow<T> *rf = new RequestFlow<T>(source);
      source >> rf;
      _requestables.push_back(rf);
      return *rf;
    }*/

  template <class T> LambdaSource<T> &operator>>(LambdaSource<T> &source) {
    _requestables.push_back(&source);
    return source;
  }

  template <class T> ValueSource<T> &operator>>(ValueSource<T> &source) {
    _requestables.push_back(&source);
    return source;
  }

  template <class T> ValueFlow<T> &operator>>(ValueFlow<T> &source) {
    _requestables.push_back(&source);
    return source;
  }

  template <class T> RefSource<T> &operator>>(RefSource<T> &source) {
    _requestables.push_back(&source);
    return source;
  }
  /*
    Poller &operator()(Requestable &rq) {
      _requestables.push_back(&rq);
      return *this;
    }*/
};

Log logger(1024);
// ---------------------------------------------- THREAD
Thread thisThread("main");
Thread ledThread("led");
Thread mqttThread("mqtt");
Thread workerThread("worker");

//  --------------------------------------------- ACTOR
#define PIN_LED 2

LedBlinker led(ledThread, PIN_LED, 301);

#ifdef MQTT_SERIAL
#include <MqttSerial.h>
MqttSerial mqtt(mqttThread);
#else
#include <MqttWifi.h>
#include <Wifi.h>
Wifi wifi(mqttThread);
MqttWifi mqtt(mqttThread);
MqttOta mqttOta;
#endif

#ifdef US
#include <UltraSonic.h>
Connector uextUs(US);
UltraSonic ultrasonic(thisThread, &uextUs);
#endif

#ifdef GPS
#include <Neo6m.h>
Connector uextGps(GPS);
Neo6m gps(thisThread, &uextGps);
#endif

#ifdef DWM1000_TAG
#include <DWM1000_Tag.h>
#endif

#ifdef REMOTE
#include <Remote.h>
Remote remote(thisThread);
#endif

#ifdef MOTOR
#include <Motor.h>
#include <RotaryEncoder.h>
Connector uextMotor(MOTOR);
#endif

#ifdef SERVO
#include <Servo.h>
Connector uextServo(SERVO);
#endif

// ---------------------------------------------- system properties
ValueSource<std::string> systemBuild("NOT SET");
ValueSource<std::string> systemHostname("NOT SET");
ValueSource<bool> systemAlive = true;
LambdaSource<uint32_t> systemHeap([]() { return Sys::getFreeHeap(); });
LambdaSource<uint64_t> systemUptime([]() { return Sys::millis(); });
Poller poller(mqttThread);

#ifdef GPIO_TEST
#include <HardwareTester.h>
HardwareTester hw;
#endif

#ifdef STEPPER
#include <Stepper.h>
Connector uextStepper(STEPPER);
Stepper stepper(workerThread, uextStepper);
#endif

#ifdef STEPPER_SERVO
#include <As5600.h>
#include <StepperServo.h>
Connector uextAs5600(2);
As5600 as5600(uextAs5600);
Connector uextStepperServo(STEPPER_SERVO);
StepperServo stepperServo(workerThread, uextStepperServo, as5600);
#endif

#ifdef HWTIMER
#include <HwTimer.h>
HwTimer hwTimer(32);
#endif

#ifdef COMPASS
#include <Compass.h>
Connector uextCompass(COMPASS);
Compass compass(workerThread, uextCompass);
#endif

#ifdef COMMAND
#include <Cli.h>
UART &uart0 = UART::create(0, 1, 3);
Cli cli(uart0);
#endif

#ifdef STM32
#include <Stm32.h>
Thread stm32Thread("stm32");
Stm32 stm32(stm32Thread, 17, 16, 5, 18);
#endif

#ifdef SWD
#include <Swd.h>
Thread stm32Thread("stm32");
Swd swd(stm32Thread, 13, 14, 12);
#endif

class EchoTest : public Actor {
public:
  TimerSource trigger;
  ValueSource<uint64_t> counter;
  ValueSource<uint32_t> delta;
  EchoTest(Thread &thread)
      : Actor(thread), trigger(thread, 1000, true, "trigger"){};
  void init() {
    trigger >> [&](const TimerMsg &tm) {
      INFO(" send ");
      counter = Sys::millis();
    };
    counter >> mqtt.toTopic<uint64_t>("echo/output");
    delta >> mqtt.toTopic<uint32_t>("echo/delta");
    mqtt.fromTopic<uint64_t>("src/gps/echo/output") >> [&](const uint64_t in) {
      delta = Sys::millis() - in;
      INFO(" it took %u msec ", delta());
    };
  }
};

EchoTest echoTest(thisThread);

extern "C" void app_main(void) {
  //    ESP_ERROR_CHECK(nvs_flash_erase());

#ifdef HOSTNAME
  Sys::hostname(S(HOSTNAME));
#else
  std::string hn;
  union {
    uint8_t macBytes[6];
    uint64_t macInt;
  };
  macInt = 0L;
  if (esp_read_mac(macBytes, ESP_MAC_WIFI_STA) != ESP_OK)
    WARN(" esp_base_mac_addr_get() failed.");
  string_format(hn, "ESP32-%d", macInt & 0xFFFF);
  Sys::hostname(hn.c_str());
#endif
  systemHostname = Sys::hostname();
  systemBuild = __DATE__ " " __TIME__;
  INFO("%s : %s ", Sys::hostname(), systemBuild().c_str());

  led.init();
#ifdef MQTT_SERIAL
  mqtt.init();
  echoTest.init();

#else
  wifi.init();
  mqtt.init();

  wifi.connected >> mqtt.wifiConnected;
  //-----------------------------------------------------------------  WIFI
  // props
  poller >> wifi.macAddress >> mqtt.toTopic<std::string>("wifi/mac");
  poller >> wifi.ipAddress >> mqtt.toTopic<std::string>("wifi/ip");
  poller >> wifi.ssid >> mqtt.toTopic<std::string>("wifi/ssid");
  poller >> wifi.rssi >> mqtt.toTopic<int>("wifi/rssi");
  mqtt.blocks >> mqttOta.blocks;
#endif
  mqtt.connected >> led.blinkSlow;
  mqtt.connected >> poller.connected;
  //-----------------------------------------------------------------  SYS props
  poller >> systemUptime >> mqtt.toTopic<uint64_t>("system/upTime");
  poller >> systemHeap >> mqtt.toTopic<uint32_t>("system/heap");
  poller >> systemHostname >> mqtt.toTopic<std::string>("system/hostname");
  poller >> systemBuild >> mqtt.toTopic<std::string>("system/build");
  poller >> systemAlive >> mqtt.toTopic<bool>("system/alive");

  TimerSource logTimer(thisThread, 5000, true);

  logTimer >> ([](const TimerMsg &tm) {
    INFO(
        " ovfl : %u busyPop : %u busyPush : %u threadQovfl : %u  CAS push : %u "
        "pop : %u retries : %u",
        stats.bufferOverflow, stats.bufferPopBusy, stats.bufferPushBusy,
        stats.threadQueueOverflow, stats.bufferPushCasFailed,
        stats.bufferPopCasFailed, stats.bufferCasRetries);
  });

#ifdef COMMAND
  cli.init();
#endif

#ifdef STM32
  stm32.init();
  stm32.wiring();
  mqtt.blocks >> stm32.ota;
  stm32.message >> mqtt.toTopic<std::string>("stm32/log");
  stm32.startAddress == mqtt.topic<uint32_t>("stm32/startAddress");
  stm32.baudrate == mqtt.topic<uint32_t>("stm32/baudrate");
#endif

#ifdef SWD
  TimerSource swdTimer(thisThread, 2000, true, "swd");

  swd.init();
  mqtt.blocks >> swd.ota;
  swdTimer >> [](const TimerMsg &tm) { swd.test(); };
#endif

#ifdef GPIO_TEST
  hw.gpioTest();
  hw.pwmFrequency = 10000;
  hw.captureNumberOfPulse = 249;
  hw.mcpwmTest();
  hw.captureTest();

  TimerSource pulser(thisThread, 10, true);
  pulser >> ([](const TimerMsg &tm) {
    static int i = 0;
    int pwm = i % 200;
    if (pwm > 100)
      pwm = 200 - i;
    hw.pwm(50);
    if (i++ == 200)
      i = 0;
  });

  TimerSource regTimer(thisThread, 1000, true, "reg");
  LambdaFlow<TimerMsg, MqttMessage> regFlow;
  regFlow.lambda([](MqttMessage &mq, const TimerMsg &tm) {
    static int cnt = 0;
    cnt++;
    if (hw.regs[cnt].name == 0)
      cnt = 0;
    mq.topic = hw.regs[cnt].name;
    Register reg(mq.topic.c_str(), hw.regs[cnt].format);
    reg.value(*hw.regs[cnt].address);
    reg.format(mq.message);
    return 0;
  });
  regTimer >> regFlow;
  regFlow >> mqtt.outgoing;

  Sink<uint32_t> sinker(20);
  sinker.async(thisThread, [](const uint32_t &cpt) {
    INFO("count : %u , freq : %f Hz", cpt,
         (160000000.0 / cpt) * (hw.captureNumberOfPulse + 1));
  });
  hw.capts >> sinker;
#endif

#ifdef HWTIMER
  hwTimer.init();
  hwTimer.divider == mqtt.topic<uint32_t>("hwtimer/divider");
  hwTimer.ticks == mqtt.topic<uint32_t>("hwtimer/ticks");
  hwTimer.intervalSec == mqtt.topic<double>("hwtimer/intervalSec");
  hwTimer.autoReload == mqtt.topic<bool>("hwtimer/autoReload");
#endif

#ifdef COMPASS
  compass.init();
  compass.x >> mqtt.toTopic<int32_t>("compass/x");
  compass.y >> mqtt.toTopic<int32_t>("compass/y");
  compass.z >> mqtt.toTopic<int32_t>("compass/z");
  compass.status >> mqtt.toTopic<int32_t>("compass/status");
//    poller(compass.x)(compass.y)(compass.z);
#endif

#ifdef US
  ultrasonic.init();
  ultrasonic.distance >> mqtt.toTopic<int32_t>("us/distance");
#endif

#ifdef GPS
  gps.init(); // no thread , driven from interrupt
  gps >> mqtt.outgoing;
#endif

#ifdef REMOTE
  remote.init();
  mqtt.fromTopic<bool>("remote/ledLeft") >> remote.ledLeft;   // timer driven
  mqtt.fromTopic<bool>("remote/ledRight") >> remote.ledRight; // timer driven
  remote.buttonLeft >>
      mqtt.toTopic<bool>("remote/buttonLeft"); // change and timer driven
  remote.buttonRight >> mqtt.toTopic<bool>("remote/buttonRight");
  remote.potLeft >> mqtt.toTopic<int>("remote/potLeft");
  remote.potRight >> mqtt.toTopic<int>("remote/potRight");
#endif

#ifdef MOTOR
  RotaryEncoder &rotaryEncoder = *new RotaryEncoder(
      thisThread, uextMotor.toPin(LP_SCL), uextMotor.toPin(LP_SDA));
  Motor &motor = *new Motor(
      thisThread, &uextMotor); // cannot init as global var because of NVS
  INFO(" init motor ");
  motor.watchdogTimer.interval(2000);
  mqtt.fromTopic<bool>("motor/watchdogReset") >> motor.watchdogReset;

  rotaryEncoder.init();
  poller >> rotaryEncoder.isrCounter >>
      mqtt.toTopic<uint32_t>("motor/isrCounter");

  motor.init();
  rotaryEncoder.rpmMeasured >> motor.rpmMeasured;

  motor.pwm >> mqtt.toTopic<float>("motor/pwm");
  motor.rpmMeasured2 >> mqtt.toTopic<int>("motor/rpmMeasured");
  motor.rpmTarget == mqtt.topic<int>("motor/rpmTarget");

  poller >> motor.KI >> mqtt.toTopic<float>("motor/KI");
  poller >> motor.KP == mqtt.topic<float>("motor/KP");
  poller >> motor.KD == mqtt.topic<float>("motor/KD");
  poller >> motor.current >> mqtt.toTopic<float>("motor/current");

  poller >> motor.deviceState >> mqtt.toTopic<int>("motor/state");
  poller >> motor.deviceMessage >> mqtt.toTopic<std::string>("motor/message");
#endif

#ifdef SERVO
  Servo &servo = *new Servo(thisThread, &uextServo);
  servo.watchdogTimer.interval(3000);
  servo.init();
  mqtt.fromTopic<bool>("servo/watchdogReset") >> servo.watchdogReset;
  poller >> servo.pwm >> mqtt.toTopic<float>("servo/pwm");
  poller >> servo.adcPot >> mqtt.toTopic<int>("servo/adcPot");
  poller >> servo.angleMeasured >> mqtt.toTopic<int>("servo/angleMeasured");
  poller >> servo.KI == mqtt.topic<float>("servo/KI");
  poller >> servo.KP == mqtt.topic<float>("servo/KP");
  poller >> servo.KD == mqtt.topic<float>("servo/KD");
  poller >> servo.current == mqtt.topic<float>("servo/current");
  poller >> servo.angleTarget == mqtt.topic<int>("servo/angleTarget");
  poller >> servo.deviceState >> mqtt.toTopic<int>("servo/state");
  poller >> servo.deviceMessage >> mqtt.toTopic<std::string>("servo/message");
#endif

#ifdef STEPPER
  stepper.init();
  // stepper.watchdogTimer.interval(3000);
  mqtt.topic<int>("stepper/angleTarget") == stepper.angleTarget;
  poller >> stepper.stepMeasured >> mqtt.toTopic<int>("stepper/stepMeasured");
  stepper.stepTarget >> Cache<int>::nw(mqttThread, 200, 2000) >>
      mqtt.toTopic<int>("stepper/stepTarget");
#endif

#ifdef STEPPER_SERVO
  as5600.init();
  stepperServo.init();
  stepperServo.watchdogTimer.interval(2000);
  mqtt.fromTopic<bool>("stepper/watchdogReset") >> stepperServo.watchdogReset;

  stepperServo.stepTarget >> Cache<int>::nw(mqttThread, 300, 1000) >>
      mqtt.toTopic<int>("stepper/stepTarget");
  stepperServo.angleMeasured >> Cache<int>::nw(mqttThread, 300, 1000) >>
      mqtt.toTopic<int>("stepper/angleMeasured");
  poller >> stepperServo.angleTarget == mqtt.topic<int>("stepper/angleTarget");

  stepperServo.errorCount >> Cache<int>::nw(mqttThread, 500, 2000) >>
      mqtt.toTopic<int>("stepper/errorCount");

  stepperServo.stepsPerRotation == mqtt.topic<int>("stepper/stepsPerRotation");
  poller >> stepperServo.deviceState >> mqtt.toTopic<int>("stepper/state");
  poller >> stepperServo.deviceMessage >>
      mqtt.toTopic<std::string>("stepper/message");
#endif

#ifdef DWM1000_TAG
  DWM1000_Tag &tag =
      *(new DWM1000_Tag(workerThread, new Connector(DWM1000_TAG)));
  tag.preStart();
  tag.mqttMsg >> mqtt.outgoing;
  //    tag.blink >> ledBlue.pulse;
  poller >> tag.blinks >> mqtt.toTopic<uint32_t>("tag/blinks");
  poller >> tag.polls >> mqtt.toTopic<uint32_t>("tag/polls");
  poller >> tag.resps >> mqtt.toTopic<uint32_t>("tag/resps");
  poller >> tag.finals >> mqtt.toTopic<uint32_t>("tag/finals");
  poller >> tag.interruptCount >> mqtt.toTopic<uint32_t>("tag/interrupts");
  poller >> tag.errs >> mqtt.toTopic<uint32_t>("tag/errs");
  poller >> tag.timeouts >> mqtt.toTopic<uint32_t>("tag/timeouts");
#endif
  ledThread.start();
  mqttThread.start();
  workerThread.start();
#ifdef STM32
  stm32Thread.start();
#endif
  thisThread.run(); // DON'T EXIT , local variable will be destroyed
}
