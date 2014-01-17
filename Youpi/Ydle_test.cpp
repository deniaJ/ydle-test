// Ydle.cpp
//
// Ydle implementation for Arduino
// See the README file in this directory for documentation
// For changes, look at Ydle.h
// 
// Author: Manuel Esteban AKA Yaug
// Contributors: Matthieu Desgardin AKA Zescientist,Yargol AKA Yargol :-)
// WebPage: http://www.ydle.fr/index.php
// Contact: http://forum.ydle.fr/index.php
// Licence: CC by sa (http://creativecommons.org/licenses/by-sa/3.0/fr/)
// Id: Ydle.cpp, v0.5, 2013/09/24
// Pll function inspired on VirtualWire library

#include "TimerOne.h"

#include "Ydle_test.h"
#include <avr/eeprom.h>

const PROGMEM prog_uchar _atm_crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


static Frame_t g_m_receivedframe;  // received frame
static uint8_t m_data[MAX_SIZE_FRAME]; // data + crc

// Le num�ro de la broche IO utilis�e pour le module r�cepteur
static uint8_t pinRx = 2;

// Le num�ro de la broche IO utilis�e pour le module �metteur
static uint8_t pinTx = 10;

// Le num�ro de la broche IO utilis�e pour la Led de statut
static uint8_t pinLed = 13;

// Permet la recopie du signal sur une sortie pour v�rification � l'oscilloscope
static uint8_t pinCop = 8;

// Le num�ro de la broche IO utilis�e pour l'installation du boutton de resettage
static uint8_t pinButton = 2;

// L'information sur le type de transmission (requete, ACK,...)
static uint8_t transmissionType = 0;

// Indique si le bouton de reset est press�
static uint8_t buttonState = false;

// Le tableau contenant l'octet de start
static unsigned char start_bit2 = 0b01000010;

// Pour activer le mode d�bug
static uint8_t debugActivated = false;

// Disponibilit� d'un sample
volatile uint8_t sample_value = 0;

// Nombre de samples sur la p�riode en cours
volatile uint8_t sample_count = 1;

// La valeur du dernier sample re�u
volatile uint8_t last_sample_value = 0;

// La rampe PLL, varie entre 0 et 79 sur les 8 samples de chaque p�riode de bit
// Quand la PLL est synchronis�e, la transition de bit arrive quand la rampe vaut 0
static uint8_t pll_ramp = 0;

// La somme des valeurs des samples. si inf�rieur � 2 "1" samples dans le cycle de PLL
// le bit est d�clar� comme 0, sinon � 1
static uint8_t sample_sum = 0;

// Les 16 derniers bits re�us, pour rep�rer l'octet de start
static uint16_t rx_bits = 0;

// Temps du premier sample de chaque bit
volatile unsigned long t_start = 0;

// Flag pour indiquer la bonne r�ception du message de start
static uint8_t rx_active = 0;

// Le d�bit de transfert en bits/secondes
#define YDLE_SPEED 1000
//static long speed = 1000;

// La p�riode d'un bit en microseconds
#define YDLE_TPER 1000000/YDLE_SPEED

// La fr�quence de prise de samples
#define YDLE_FBIT YDLE_TPER/8

// La valeur du dernier bit r�cup�r�
static uint8_t bit_value = 0;

// Le nombre de bits r�cup�r�s
static uint8_t bit_count = 0;

// Id sender re�ue
static uint8_t sender = 0;

// Id receptor re�ue
static uint8_t receptor = 0;

// Info type re�ue
static uint8_t type = 0;

// Info parit� re�ue
static uint8_t parite = false;

// Info taille re�ue
static uint8_t taille = 0;

// Data re�ues
static unsigned int data = 0;

// Nombre d'octets re�us
static int rx_bytes_count = 0;

// Disponibilit� de la taille de trame
static uint8_t length_ok = 0;

// Si le message est complet
static uint8_t rx_done = 0;
static uint8_t frameReadyToBeRead = false;
volatile uint8_t transmission_on = false;



uint8_t Ydle_test::crc8(const uint8_t* buf, uint8_t length) {
	// The inital and final constants as used in the ATM HEC.
	const uint8_t initial = 0x00;
	const uint8_t final = 0x55;
	uint8_t crc = initial;
	while (length) {
		crc = pgm_read_byte_near(_atm_crc8_table + (*buf ^ crc));
		buf++;
		length--;
	}
	return crc ^ final;
}

unsigned char Ydle_test::computeCrc(Frame_t* frame){
	unsigned char *buf, crc;
	int a,j;

	buf = (unsigned char*)malloc(frame->taille+3);
	memset(buf, 0x0, frame->taille+3);

	buf[0] = frame->sender;
	buf[1] = frame->receptor;
	buf[2] = frame->type;
	buf[2] = buf[2] << 5;
	buf[2] |= frame->taille;

	for(a=3, j=0 ;j < frame->taille - 1;a++, j++){
		buf[a] = frame->data[j];
	}
	crc = crc8(buf,frame->taille+2);
	free(buf);
	return crc;
}

void Ydle_test::rfReceiveTask(){
	unsigned char crc_p;
	
	if(frameReadyToBeRead){
		crc_p = computeCrc(&g_m_receivedframe);
		if(crc_p != g_m_receivedframe.crc)
		{
#ifdef _YDLE_DEBUG
			if(debugActivated)
				log("crc error!!!!!!!!!");
#endif // _YDLE_DEBUG
		}
		else
		{
#ifdef _YDLE_DEBUG
			Serial.println("Frame ready to be handled");
#endif // _YDLE_DEBUG
			// We receive a CMD so trait it
			if(g_m_receivedframe.type == TYPE_CMD)
			{
				HandleReceiveCommand(&g_m_receivedframe);
#ifdef _YDLE_DEBUG
				printFrame(&g_m_receivedframe);
#endif // _YDLE_DEBUG
			}else if(g_m_receivedframe.type == TYPE_ACK){
				Serial.println("ACK received");
			}else{
#ifdef _YDLE_DEBUG
				printFrame(&g_m_receivedframe);
#endif
				// Send the frame to the callback function
				if(this->_callback_set)
					this->callback(&g_m_receivedframe);
			}
			// Frame handled			
		}
		frameReadyToBeRead = false;
	}
}


// Initialisation des IO avec des valeurs entr�es par l'utilisateur
Ydle_test::Ydle_test(int rx, int tx, int button)
{
	m_bLnkSignalReceived=false;
	m_initializedState = false;
	this->_callback_set = false;
	//readEEProm();
	pinMode(rx, INPUT);
  	pinRx = rx;
  	pinMode(tx, OUTPUT);
  	pinTx = tx;
  	pinMode(button, INPUT);
  	pinButton = button;
	pinMode(pinLed, OUTPUT); 

}


// Initialisation des IO avec les valeurs par d�faut
Ydle_test::Ydle_test()
{
	m_bLnkSignalReceived=false;
	m_initializedState = false;
	this->_callback_set = false;
	readEEProm();
  	pinMode(pinRx, INPUT);
  	pinMode(pinTx, OUTPUT);
  	pinMode(pinButton, INPUT);
  	pinMode(pinLed, OUTPUT);
  	// Permet la recopie du signal sur une sortie pour v�rification � l'oscilloscope
  	//pinMode(pinCop, OUTPUT);
	  
}

// Affiche les logs sur la console s�rie
void Ydle_test::log(String msg)
{
#if not defined( __AVR_ATtiny85__ ) or defined(_YDLE_DEBUG)
	if(debugActivated)
		Serial.println(msg);
#endif
}
// Affiche les logs sur la console s�rie
void Ydle_test::log(String msg,int i)
{
#if not defined( __AVR_ATtiny85__ ) or defined(_YDLE_DEBUG)
	if(debugActivated)
		Serial.print(msg);
		Serial.println(i);
#endif
}

// Synchronise l'AGC, envoie l'octet de start puis transmet la trame
void Ydle_test::send(Frame_t *frame)
{
	int i = 0,j = 0;

	digitalWrite(pinLed, HIGH);   // on allume la Led pour indiquer une �mission

	// calcul crc
	frame->taille++; // add crc BYTE
	frame->crc = computeCrc(frame);

#ifdef _YDLE_DEBUG
	printFrame(frame);
#endif
	// From now, we are ready to transmit the frame

	if(!rx_active){
		// Wait that the current transmission finish
		delay(500);
	}
	// So, stop the interruption while sending
	Timer1.stop();
	
 	// Sequence de bits pour réglages de l'AGC
 	for (i=0; i<16; i++)
 	{
 		digitalWrite(pinTx, HIGH);
 		delayMicroseconds(YDLE_TPER);     // un bit � l'�tat haut
 		digitalWrite(pinTx, LOW);
 		delayMicroseconds(YDLE_TPER);     // un bit � l'�tat bas
	}
	
 	// Octet de start annoncant le départ du signal au recepteur 	
	for (i = 7; i>=0; i--)
	{
		digitalWrite(pinTx, (start_bit2 & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
		digitalWrite(pinTx, !(start_bit2 & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
 	}

	// Frame sending
	// TODO : Inverser sender and receptor
	for(i = 7; i>=0; i--){
		digitalWrite(pinTx, (frame->sender & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
		digitalWrite(pinTx, !(frame->sender & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
	}
	for(i = 7; i>=0; i--){
		digitalWrite(pinTx, (frame->receptor & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
		digitalWrite(pinTx, ~(frame->receptor & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
	}
	for(i = 2; i>=0; i--){
		digitalWrite(pinTx, (frame->type & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
		digitalWrite(pinTx, !(frame->type & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
	}	
	for(i = 4; i>=0; i--){
		digitalWrite(pinTx, (frame->taille & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
		digitalWrite(pinTx, !(frame->taille & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
	}
	for(i=0;i<frame->taille-1;i++){
		for(j = 7; j>=0; j--){
			digitalWrite(pinTx, (frame->data[i] & 1<<j));   // Pulsation � l'�tat haut
			delayMicroseconds(YDLE_TPER);      // t_per
			digitalWrite(pinTx, !(frame->data[i] & 1<<j));   // Pulsation � l'�tat haut
			delayMicroseconds(YDLE_TPER);      // t_per
		}
	}
	for(i = 7; i>=0; i--){
		digitalWrite(pinTx, (frame->crc & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
		digitalWrite(pinTx, !(frame->crc & 1<<i));   // Pulsation � l'�tat haut
		delayMicroseconds(YDLE_TPER);      // t_per
	}

	digitalWrite(pinTx, LOW);
	digitalWrite(pinLed, LOW);

	// Restart the timer task
	Timer1.initialize(128);
	Timer1.attachInterrupt(timerInterrupt);
}

// Comparaison du signal re�u et du signal de r�f�rence
bool Ydle_test::checkSignal(Frame_t *frame)
{
	if(frame->sender==m_Config.IdMaster && frame->receptor==m_Config.IdNode)
		return true;
 	else
		return false;
}

// Used to read the configuration 
void Ydle_test::ReadConfig()
{
	readEEProm();
}

// lire en m�moire EEProm du signal de r�f�rence
void Ydle_test::readEEProm()
{
	memset(&m_Config,0x0,sizeof(m_Config));
	eeprom_read_block ((void*)&m_Config, 0, sizeof(Config_t));
	if((m_Config.IdMaster!=0)&&(m_Config.IdNode!=0)){
		m_initializedState = true;
#ifdef _YDLE_DEBUG
		log("Config find in eeprom");
		log("Config.IdMaster : ", m_Config.IdMaster);
		log("Config.IdNode :", m_Config.IdNode);
#endif
	}
	else{
#ifdef _YDLE_DEBUG
		log("NODE IS NOT INIT");		
		log("No config in EEprom");
#endif
		memset(&m_Config,0,sizeof(m_Config));
	}
}

// Ecriture en m�moire EEProm du signal de r�f�rence
void Ydle_test::writeEEProm()
{
    eeprom_write_block(&m_Config,0, sizeof(m_Config));
#ifdef _YDLE_DEBUG
    log("Enregistrement du signal re�u comme signal de r�ference");
#endif
}

void timerInterrupt(){
	if(!transmission_on){
		if(rx_done)
		{
			rx_done = false;
		}
		sample_value = digitalRead(pinRx);
		pll2();
	}
}

void Ydle_test::init_timer(){
	Timer1.initialize(128); // set a timer of length 128 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
	Timer1.attachInterrupt( timerInterrupt ); // attach the service routine here
}

void pll2()
{
	sample_count ++;
	// On additionne chaque sample et on incr�mente le nombre du prochain sample
	if (sample_value){
		sample_sum++;
	}
	
	// On v�rifie s'il y a eu une transition de bit
	if (sample_value != last_sample_value){
		// Transition, en avance si la rampe > 40, en retard si < 40
		if(pll_ramp < 80){
			pll_ramp += 11;
		} 
		else{
			pll_ramp += 29;
		}
		last_sample_value = sample_value;
	}
	else{
		// Si pas de transition, on avance la rampe de 20 (= 80/4 samples)
		pll_ramp += 20;
	}
	
	// On v�rifie si la rampe � atteint son maximum de 80
	if (pll_ramp >= 160)
	{
		//t_start = micros();
		// On ajoute aux 16 derniers bits re�us rx_bits, MSB first
		// On stock les 16 derniers bits
		rx_bits <<= 1;
		
		// On v�rifie la somme des samples sur la p�riode pour savoir combien �tait � l'�tat haut
		// S'ils �taient < 2, on d�clare un 0, sinon un 1;
		if (sample_sum >= 5){
			rx_bits |= 0x1;
			bit_value = 1;
		}
		else{
			rx_bits |= 0x0;
			bit_value = 0;
		}
		pll_ramp -= 160; // On soustrait la taille maximale de la rampe � sa valeur actuelle
		sample_sum = 0; // On remet la somme des samples � 0 pour le prochain cycle
		sample_count = 1; // On r�-initialise le nombre de sample

		// Si l'on est dans le message, c'est ici qu'on traite les donn�es
		if (rx_active){
			bit_count ++;
			// On r�cup�re les bits et on les places dans des variables
			// 1 bit sur 2 avec Manchester
			if (bit_count % 2 == 1){
				if (bit_count < 16){
					// Les 8 premiers bits de donn�es
					sender <<= 1;
					sender |= bit_value;
				}
				else if (bit_count < 32){
					// Les 8 bits suivants
					receptor <<= 1;
					receptor |= bit_value;
				}
				else if (bit_count < 38){
					// Les 3 bits de type
					type <<= 1;
					type |= bit_value;
				}
				else if (bit_count < 48){
					// Les 5 bits de longueur de trame
					rx_bytes_count <<= 1;
					rx_bytes_count |= bit_value;
				}
				else if ((bit_count-48) < (rx_bytes_count * 16)){
					// les donn�es
					length_ok = 1;
					m_data[(bit_count-48)/16] <<= 1;
					m_data[(bit_count-48)/16]|= bit_value;
				}
			}
			
			// Quand on a re�u les 24 premiers bits, on connait la longueur de la trame
			// On v�rifie alors que la longueur semble logique
			if (bit_count >= 48)
			{
				// Les bits 19 � 24 informent de la taille de la trame
				// On les v�rifie car leur valeur ne peuvent �tre < � 1 et > � 30 + 1 pour le CRC
				if (rx_bytes_count < 1 || rx_bytes_count > 30)
				{
#ifdef _YDLE_DEBUG
					Serial.println("error!");
#endif
					// Mauvaise taille de message, on r�-initialise la lecture
					rx_active = false;
					sample_count = 1;
					bit_count = 0;
					length_ok = 0;
					sender = 0;
					receptor = 0;
					type = 0;
					parite = 0;
					taille = 0;
					data = 0;
					memset(m_data,0,sizeof(m_data));
					t_start = micros();
					return;
				}
			}

			// On v�rifie si l'on a re�u tout le message
			if ((bit_count-48) >= (rx_bytes_count*16) && (length_ok == 1))
			{
#ifdef _YDLE_DEBUG
				Serial.println("complete");
#endif

				rx_active = false;
				g_m_receivedframe.sender = sender;
				g_m_receivedframe.receptor = receptor;
				g_m_receivedframe.type = type;
				g_m_receivedframe.taille = rx_bytes_count; // data + crc
				memcpy(g_m_receivedframe.data,m_data,rx_bytes_count-1); // copy data len - crc
				g_m_receivedframe.crc=m_data[rx_bytes_count-1];
				// May be an array ?
				frameReadyToBeRead = true;

				length_ok = 0;
				sender = 0;
				receptor = 0;
				type = 0;
				taille = 0;
				memset(m_data,0,sizeof(m_data));
			}

		}

		// Pas dans le message, on recherche l'octet de start
		else if (rx_bits == 0x6559)
		{
#ifdef _YDLE_DEBUG
			Serial.println("start");
#endif
			// Octet de start, on commence � collecter les donn�es
			rx_active = true;
			bit_count = 0;
			rx_bytes_count = 0;
		}
	}
}

// Do something with a received Command
void Ydle_test::HandleReceiveCommand(Frame_t *frame)
{
	int litype;
	long livalue;

	//Special case for CMD_LINK
	if(this->extractData(frame, 0,litype,livalue)==1)
	{
#ifdef _YDLE_DEBUG
		if(debugActivated)
			log("Type of  Message Received ",litype);
#endif
		//A node ask to link us	
		if(litype == CMD_LINK)
		{
#ifdef _YDLE_DEBUG
			if(debugActivated)
				log("Link received ",litype);
#endif
			// we are not already linked
			if(m_initializedState == false)
			{
				m_Config.IdNode=frame->receptor;
				m_Config.IdMaster=frame->sender;
				m_bLnkSignalReceived=true;
                                m_initializedState=true;
				//writeEEProm();
			}
		}
		
		//Master ask to reset or configuration
		if(litype==CMD_RESET)
		{
				
			if (checkSignal(frame))
			{
#ifdef _YDLE_DEBUG
				if(debugActivated)
					log("Reset received ",litype);
#endif
				m_Config.IdNode=0;
				m_Config.IdMaster=0;
				//writeEEProm();
				m_initializedState=false;
			}
		}
		
		// send ACK if frame is for us.	
		if(checkSignal(frame))
		{
			delay(1000);
#ifdef _YDLE_DEBUG
			log("**************Send ACK********************");
#endif
			Frame_t response;
			memset(&response, 0x0, sizeof(response));
			dataToFrame(&response, TYPE_ACK);	// Create a new ACK Frame
#ifdef _YDLE_DEBUG
			printFrame(&response);
			Serial.println("End send ack");
#endif
			send(&response);

		}
	}
}

// Fonction qui cr�e une trame avec les infos fournies
void Ydle_test::dataToFrame(Frame_t *frame, unsigned long destination, unsigned long sender, unsigned long type)
{
	frame->sender = sender;
	frame->receptor = destination;
	frame->type = type;
	frame->taille = 0;
	frame->crc = 0;
	memset(frame->data,0,sizeof(frame->data));
} 

// Fonction qui cr�e une trame avec un type fournie
void Ydle_test::dataToFrame(Frame_t *frame, unsigned long type)
{
	frame->sender = m_Config.IdNode;
	frame->receptor = m_Config.IdMaster;
	frame->type = type;
	frame->taille = 0;
	frame->crc = 0;
	memset(frame->data, 0x0, sizeof(frame->data));
} 

// Fonction qui retourne "true" si la Node est initialis�e
bool Ydle_test::initialized()
{
	return m_initializedState;
}


// Fonction qui retourne "true" si la Node est initialis�e
bool Ydle_test::resetButton()
{
	return (digitalRead(pinButton) == HIGH);
}

// Fonction pour activer le mode d�bug
void Ydle_test::debugMode()
{
	debugActivated = true;
}


int Ydle_test::isSignal()
{
	return rx_active;
}


/*bool Ydle_test::isDone()
{
	return (checkSignal() && rx_done);	
}*/

// ----------------------------------------------------------------------------
/**
	   Function: printFrame
	   Inputs:  Frame_t trame  frame to log
				int data

	   Outputs: 
		Log a frame if debug activated
*/
// ----------------------------------------------------------------------------
void Ydle_test::printFrame(Frame_t *trame)
{
#if not defined( __AVR_ATtiny85__ ) or defined (_YDLE_DEBUG)
	// if debug
	if(debugActivated)
	{
		char sztmp[255];
		
		log("-----------------------------------------------");
		sprintf(sztmp,"Emetteur :%d",trame->sender);
		log(sztmp);

		sprintf(sztmp,"Recepteur :%d",trame->receptor);
		log(sztmp);

		sprintf(sztmp,"Type :%d",trame->type);
		log(sztmp);

		sprintf(sztmp,"Taille :%d",trame->taille);
		log(sztmp);

		sprintf(sztmp,"CRC :%d",trame->crc);
		log(sztmp);

		sprintf(sztmp,"Data Hex: ");
		for (int a=0;a<trame->taille-1;a++)
			sprintf(sztmp,"%s 0x%02X",sztmp,trame->data[a]);
		log(sztmp);

		sprintf(sztmp,"Data Dec: ");
		for (int a=0;a<trame->taille-1;a++)
			sprintf(sztmp,"%s %d",sztmp,trame->data[a]);
		log(sztmp);
		log("-----------------------------------------------");
	}
#endif
}

// ----------------------------------------------------------------------------
/**
	   Function: addCmd
	   Inputs:  int type type of data
				int data

	   Outputs: 

*/
// ----------------------------------------------------------------------------
void Ydle_test::addCmd(Frame_t *frame, int type,int data)
{
	frame->data[frame->taille]=type<<4;
	frame->data[frame->taille+1]=data;
	frame->taille+=2;
}



// ----------------------------------------------------------------------------
/**
	   Function: extractData
	   Inputs:  int index: index de la value recherche (0..29)
				int itype: en retour type de la value
				int ivalue: en retour, value

	   Outputs: 1 value trouve,0 non trouve,-1 no data

*/
// ----------------------------------------------------------------------------
int Ydle_test::extractData(Frame_t *frame, int index, int &itype, long &ivalue)
{
	uint8_t* ptr;
	bool bifValueisNegativ=false;
	int iCurrentValueIndex=0;
	bool bEndOfData=false;
	int  iModifType=0;
	int  iNbByteRest;

	ptr=frame->data;
	
	if(frame->taille <2) // Min 1 byte of data with the 1 bytes CRC always present, else there is no data
	 return -1;
	
	iNbByteRest= frame->taille-1;
	while (!bEndOfData)
	{
		itype=*ptr>>4;
		bifValueisNegativ=false;
		
		// This is a very ugly code :-( Must do something better
		if( frame->type==TYPE_CMD)
		{
			// Cmd type if always 12 bits signed value
			iModifType=DATA_DEGREEC;
		}
		else if(frame->type==TYPE_ETAT)
		{
			iModifType=itype;
		}
		else
		{
			iModifType=itype;
		}

		switch(iModifType)
		{
			// 4 bits no signed
			case DATA_ETAT :    
				ivalue=*ptr&0x0F;
				ptr++;
				iNbByteRest--;
			break;	

			// 12 bits signed
			case DATA_DEGREEC:  
			case DATA_DEGREEF : 
			case DATA_PERCENT : 
			case DATA_HUMIDITY: 
				if(*ptr&0x8)
					bifValueisNegativ=true;
				ivalue=*ptr&0x0F<<8;
				ptr++;
				ivalue+=*ptr;
				ptr++;
				if(bifValueisNegativ) 
					ivalue=ivalue *(-1);
				iNbByteRest-=2;	
			break;	

			// 12 bits no signed
			case DATA_DISTANCE: 
			case DATA_PRESSION: 
				ivalue=(*ptr&0x0F)<<8;
				ptr++;
				ivalue+=*ptr;
				ptr++;
				iNbByteRest-=2;	
			break;	

			// 20 bits no signed
			case DATA_WATT  :   
				ivalue=(*ptr&0x0F)<<16;
				ptr++;
				ivalue+=*ptr<<8;
				ptr++;
				ivalue+=*ptr;
				ptr++;
				iNbByteRest-=3;	
			break;	
		}
		
		if (index==iCurrentValueIndex)
			return 1;
		
		iCurrentValueIndex++;
		
		if(iNbByteRest<1)
			bEndOfData =true;;
	}

	return 0;	
}


// ----------------------------------------------------------------------------
/**
	   Function: addData
	   Inputs:  int type type of data
				long data

	   Outputs: 

*/
// ----------------------------------------------------------------------------
void Ydle_test::addData(Frame_t * frame, int type, long data)
{
	int oldindex = frame->taille;

	switch (type){
		// 4 bits no signed
		case DATA_ETAT :    
			if (frame->taille < 29)
			{
				frame->taille++;
				frame->data[oldindex]=type<<4;
				frame->data[oldindex]+=data&0x0f;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif
		break;	

		// 12 bits signed
		case DATA_DEGREEC:  
		case DATA_DEGREEF : 
		case DATA_PERCENT : 
		case DATA_HUMIDITY: 
			if (frame->taille < 28)
			{
				frame->taille += 2;
				frame->data[oldindex]=type<<4;
				if (data <0)
				{
					data=data *-1;
					frame->data[oldindex]^=0x8;
				}
				frame->data[oldindex]+=(data>>8)&0x0f;
				frame->data[oldindex+1]=data;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif
		break;	

		// 12 bits no signed
		case DATA_DISTANCE: 
		case DATA_PRESSION: 
			if (frame->taille < 28)
			{
				frame->taille += 2;
				frame->data[oldindex]=type<<4;
				frame->data[oldindex]+=(data>>8)&0x0f;
				frame->data[oldindex+1]=data;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif
		break;	

		// 20 bits no signed
		case DATA_WATT  :   
			if (frame->taille<27)
			{
				frame->taille+=3;
				frame->data[oldindex]=type<<4;
				frame->data[oldindex]+=(data>>16)&0x0f;
				frame->data[oldindex+1]=(data>>8)&0xff;
				frame->data[oldindex+2]=data;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif // _YDLE_DEBUG
		break;	
	}
}
	
void Ydle_test::attach(ydleCallbackFunction function){
	this->callback = function;
	this->_callback_set = true;
}

