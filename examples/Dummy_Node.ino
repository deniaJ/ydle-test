#include "Ydle_test.h"
#define RX_PIN 2
#define TX_PIN 10
#define BT_PIN 1

// Init de la node, 
Ydle_test y(RX_PIN, TX_PIN, BT_PIN);

void setup()
{
	y.init_timer();
	// Insérer votre code d'init aprés
		
}

void loop()
{
	y.rfReceiveTask();
	if(y.initialized()){
		// Insérer le code utilisateur ici
	}
}

