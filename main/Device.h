#ifndef DEVICE_H
#define DEVICE_H

#include <limero.h>

class Device {
public:
  typedef enum { STOPPED, RUNNING, PAUSED } DeviceState;
  TimerSource watchdogTimer;

  ValueSource<int> deviceState = PAUSED;
  ValueSource<std::string> deviceMessage;
  ValueFlow<bool> watchdogReset = true;
  Device(Thread &thr);
  ~Device();
  void run() {
    if (deviceState() == PAUSED) {
      deviceState = RUNNING;
      deviceMessage = "RUNNING";
      INFO(deviceMessage().c_str());
    }
  };
  void stop(const char *reason) {
    std::string s = "STOPPED:";
    s += reason;
    deviceMessage = s;
    deviceState = STOPPED;
    WARN(s.c_str());
  }
  void pause(const char *reason) {
    std::string s = "PAUSED:";
    s += reason;
    deviceMessage = s;
    deviceState = PAUSED;
    WARN(s.c_str());
  }
  bool isRunning() { return deviceState() == RUNNING; }
  bool isStopped() { return deviceState() == STOPPED; }
  bool isPaused() { return deviceState() == PAUSED; }
};

#endif // DEVICE_H
