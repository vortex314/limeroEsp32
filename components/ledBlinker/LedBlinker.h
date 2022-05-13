#ifndef LEDBLINKER_H
#define LEDBLINKER_H

#include <limero.h>
#include "driver/gpio.h"


class LedBlinker : public Actor {
		uint32_t _pin;
		int _on=0;

	public:
		static const int BLINK_TIMER_ID=1;
		TimerSource blinkTimer;
		Sink<TimerMsg> timerHandler;
		Sink<bool> blinkSlow;
		LedBlinker(Thread& thr,uint32_t pin, uint32_t delay);
		void init();
		void delay(uint32_t d);
		void onNext(const TimerMsg&);
};

#endif // LEDBLINKER_H
