#ifndef STEPPER_H
#define STEPPER_H
#include <Hardware.h>
#include <Pulser.h>
#include <limero.h>

class Stepper : public Actor, public Device {
  Uext &_uext;
  Pulser _pulser;
  DigitalOut &_pinDir;
  DigitalOut &_pinEnable;
  DigitalIn &_pinCenter;
  DigitalIn &_pinLeft;
  DigitalIn &_pinRight;
  int _stepUp = 0;
  int _direction = 0;

 public:
  ValueFlow<int> angleTarget = 0;
  ValueFlow<int> stepTarget = 0;
  ValueFlow<int> stepMeasured = 0;
  Sink<std::string> msg;
  Stepper(Thread &thr, Uext &uext);
  ~Stepper();
  void init();
  void holdAngle();
  void stopStepper();
  static void isrCenter(void *ptr);
  static void isrLeft(void *ptr);
  static void isrRight(void *ptr);
};

#endif  // STEPPER_H
