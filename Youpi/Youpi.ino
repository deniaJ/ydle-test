#include "Ydle_test.h"

#define DELAY_SEND 1000
Ydle_test y(2, 10, 1);

unsigned long last_send, cur_time;

// User callback to handle the new order
void dummy_callback(Frame_t *frame){
	Serial.println("Hey, i'm the callback dude !");
	Serial.print("Paquet recu de :");
	Serial.println(frame->sender);
}

void setup()
{
	Serial.begin(115200);
	Serial.println("init complete");

	y.init_timer();
	y.attach(dummy_callback);
	y.debugMode();

	cur_time = millis();
	last_send = cur_time;
}

void loop()
{
	y.rfReceiveTask();
	if(y.initialized()){
		cur_time = millis();
		if(cur_time - last_send >= DELAY_SEND){
			last_send = cur_time;
			Frame_t f;
			y.dataToFrame(&f, 1, 2, 1);
			y.addData(&f, DATA_DEGREEC, 10);
			y.send(&f);
		}
	}
}
