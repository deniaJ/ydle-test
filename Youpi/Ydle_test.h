// Ydle.h
//
// Ydle implementation for Arduino
// See the README file in this directory for documentation
//
// Author: Manuel Esteban AKA Yaug
// Contributors: Matthieu Desgardin AKA Zescientist,Yargol AKA Yargol :-)
// WebPage: http://www.ydle.fr/index.php
// Contact: http://forum.ydle.fr/index.php
// Licence: CC by sa (http://creativecommons.org/licenses/by-sa/3.0/fr/)
// Id: Ydle.h, v0.5, 2013/09/24

/// \Mainpage Ydle library for Arduino
///
/// \Installation
/// To install, unzip the library into the libraries sub-directory of your
/// Arduino application directory. Then launch the Arduino environment; you
/// should see the library in the Sketch->Import Library menu, and example
/// code in File->Sketchbook->Examples->Ydle menu.
///
///
/// \Revision History:
/// \version 0.1
///     - Original release of the Node code
///
/// \version 0.2 2013-08-20
/// 	- Creation of the library
///
/// \version 0.5 2013-09-24
/// 	- Now use Pll function to receive signal
/// 	- Partial asynchronous rewrite of the code
///		- Parity bit
/// 	- Variable frame length for more informations
///
/// To use the Ydle library, you must have:
///     #include <Ydle.h>
/// At the top of your sketch.
///

#ifndef Ydle_h
#define Ydle_h

#include <stdlib.h>
#include <Arduino.h>
#if defined(ARDUINO)
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <wiring.h>
#endif
#else // error
#error Platform not defined
#endif

#define MAX_SIZE_FRAME 64

#define TYPE_ETAT 		1 // Node send data
#define TYPE_CMD  		2 // ON/OFF sortie etc...
#define TYPE_ACK  		3 // Acquit last command
#define TYPE_ETAT_ACK 	4 // Node send data and want ACK

#define DATA_ETAT     1 // On - OFF (4bits)
#define DATA_DEGREEC  2 // Degr�e Celsius ** -204.7 � 204.7 (12bits)
#define DATA_DEGREEF  3 // Degr�e Fahrenheit ** -204.7 � 204.7 (12bits)
#define DATA_PERCENT  4 // Pourcentage ** -100% � 100% (12bits)
#define DATA_DISTANCE 5 // Distance en Cm ** 0 � 4095 (12 bits)
#define DATA_WATT     6 // Watt ** 0 � 1048575 (20bits)
#define DATA_HUMIDITY 7 // Pourcentage humidit� ** 0 � 100% (12bits)
#define DATA_PRESSION 8 // Pression en hpa 0 � 4095 (12bits)

#define CMD_LINK  0 //Link a node to the master
#define CMD_ON    1 //Send a ON command to node data = N� output
#define CMD_OFF   2 //Send a OFF command to node data = N� output
#define CMD_RESET 3 //Ask a node to reset is configuration

// D�finie un type de structure Frame_t
struct Frame_t
{
	uint8_t receptor; // 8 bytes
	uint8_t sender; // 8 bytes
	uint8_t type; // 2 bytes
	uint8_t taille;	// 3 bytes data len + crc in BYTES
	uint8_t data[30];
	uint8_t crc; // 8 bytes
};

// D�finie un type de structure Config_t
struct Config_t
{
	uint8_t IdMaster;
	uint8_t IdNode;
	uint8_t type;
};

extern "C" {
	// callback function
	typedef void (*ydleCallbackFunction)(Frame_t *frame);
}

void timerInterrupt();
void pll2();

class Ydle_test
{
private:
	// On d�clare les structures
	static Frame_t m_receivedframe;  // received frame
	Frame_t m_sendframe;	 // send frame
	Config_t m_Config;
	bool m_bLnkSignalReceived;
	// Indique si le node est initialis�
	bool m_initializedState;	
	ydleCallbackFunction callback;
	bool _callback_set;
public:
	// Le constructeur qui lance une instance avec les num�ros des pins de l'�metteur, du r�cepteur et du boutton
	// Par d�faut, le r�cepteur est en 12, l'�metteur en 10 et le boutton en 2
	Ydle_test(int rx, int tx, int button);
	
	// Le constructeur qui lance une instance avec les num�ros des pins de l'�metteur, du r�cepteur et du boutton
	// Par d�faut, le r�cepteur est en 12, l'�metteur en 10 et le boutton en 2
	Ydle_test();
	
	// Used to read the configuration
	void ReadConfig();
	
	// Envoie des verrous et des bits formant une trame
	void send(Frame_t *frame);
	
	// Ecoute le r�cepteur pour l'arriv�e d'un signal
	void listenSignal();
	
	// Cr�e une trame avec les infos donn�es en param�tre
	void dataToFrame(Frame_t *frame, unsigned long destination, unsigned long sender, unsigned long type);
	
	// Cr�e une trame avec le type
	void dataToFrame(Frame_t *frame, unsigned long type);

	// add TYPE_ETAT data
	void addData(Frame_t *frame, int type,long data);
	
	// extract any type of data from receivedsignal
	int extractData(Frame_t *frame, int index, int &itype, long &ivalue);

	// add TYPE_CMD data
	void addCmd(Frame_t *frame, int type,int data);

	// Affiche le contenue des trames re�ues
	void printFrame();
	
	// Retourne l'�tat de la Node
	bool initialized();

	// Retourne l'�tat du bouton de reset
	bool resetButton();
	
	// Pour activer le mode d�bug
	void debugMode();

	int isSignal();
	bool isDone();
	
	// CRC calculation
	unsigned char computeCrc(Frame_t *frame);
	// Launch the timer for the receive function
	void init_timer();
	// New function need to be called by the main function in order to handle the new received frame
	void rfReceiveTask();
	// Function to attach a user defined function for handle received frame
	void attach(ydleCallbackFunction function);
private:

	// Affiche les logs
	void log(String msg);

	void log(String msg,int i);

	void printFrame(Frame_t *trame);
	
	// Do something with a received Command
	void HandleReceiveCommand(Frame_t *frame);
	
	// Double un bit en code Manchester
	void sendPair(bool b);
	// Cr�e une pulsation haute de 310�s suivie d'une pulsation basse repr�sentative du bit � envoyer
	void sendBit(bool b);

	// Compare le signal re�u au signal de r�f�rence
	bool checkSignal(Frame_t *frame);
	
	// Ecriture en m�moire EEProm du signal de r�f�rence
	void writeEEProm();
	// Lecture de EEProm
	void readEEProm();

	uint8_t crc8(const uint8_t* buf, uint8_t length);
};

#endif
