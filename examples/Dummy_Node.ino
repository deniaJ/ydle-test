#include "Ydle_test.h"
#define RX_PIN 2
#define TX_PIN 10
#define BT_PIN 1

// Init de la node, 
Ydle_test y(RX_PIN, TX_PIN, BT_PIN);

void setup()
{
	y.init_timer();
	// Ins�rer votre code d'init apr�s
		
}

void loop()
{
	y.rfReceiveTask();
	if(y.initialized()){
		// Ins�rer le code utilisateur ici
	}
}

