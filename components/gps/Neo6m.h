#ifndef NEO6M_H
#define NEO6M_H
#include <Hardware.h>
#include <Log.h>
#include <limero.h>
#include <Mqtt.h>

class Neo6m : public Actor,public Source<MqttMessage> {
		Uext* _connector;
		UART& _uart;
		static void onRxd(void*);
		std::string _line;
	public:
		Neo6m(Thread& thr,Uext* connector);
		virtual ~Neo6m();
		void init();
		void handleRxd();
		void request();
};

#endif // NEO6M_H
