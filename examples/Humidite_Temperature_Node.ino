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
			// code de r�cup�ration des informations venant du capteurs
			if(dht.read(5) == DHTLIB_OK){
				
				
				// Cr�ation de la frame qui va �tre envoy�e
				Frame_t frame;
				// On y ins�re les diff�rentes valeurs dont nous avons besoin
			
				/** Nous demandons un accus� r�c�ption.
				 * La biblioth�que va s'occuper seule de l'acquitement
				 * Si aucun acquitement n'est re�u au bout d'une seconde
				 * elle proc�dera � un nouvel envois. Si au bout de 3 envois successifs
				 * aucun acquittement n'est re�u, nous consid�rons la trame comme perdue.
				 */
				y.dataToFrame(&frame, TYPE_ETAT_ACK);
			
				/** Nous ajoutons nos valeurs que nous avons r�cup�r�es aupr�s du capteur	
				 */
				y.addData(&frame, DATA_HUMIDITY, dht.humidity);
				y.addData(&frame, DATA_DEGREEC, dht.temperature);
				/** Nous proc�dons � l'envois de la trame.
				 *
				 */ 
				y.send(&frame);
			}
		}
	}
}

