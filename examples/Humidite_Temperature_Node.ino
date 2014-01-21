#include "Ydle_test.h"
#include "dht11.h"

#define RX_PIN 2
#define TX_PIN 10
#define BT_PIN 1

#define DELAY_SEND 5000

dht11 dht;

Ydle_test y(RX_PIN, TX_PIN, BT_PIN);

unsigned long last_send, cur_time;

void setup()
{
	Serial.begin(115200);
	Serial.println("init complete");

	y.init_timer();

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
			// code de récupération des informations venant du capteurs
			if(dht.read(5) == DHTLIB_OK){
				
				
				// Création de la frame qui va être envoyée
				Frame_t frame;
				// On y insére les différentes valeurs dont nous avons besoin
			
				/** Nous demandons un accusé récéption.
				 * La bibliothéque va s'occuper seule de l'acquitement
				 * Si aucun acquitement n'est reçu au bout d'une seconde
				 * elle procédera à un nouvel envois. Si au bout de 3 envois successifs
				 * aucun acquittement n'est reçu, nous considérons la trame comme perdue.
				 */
				y.dataToFrame(&frame, TYPE_ETAT_ACK);
			
				/** Nous ajoutons nos valeurs que nous avons récupérées auprés du capteur	
				 */
				y.addData(&frame, DATA_HUMIDITY, dht.humidity);
				y.addData(&frame, DATA_DEGREEC, dht.temperature);
				/** Nous procédons à l'envois de la trame.
				 *
				 */ 
				y.send(&frame);
			}
		}
	}
}

