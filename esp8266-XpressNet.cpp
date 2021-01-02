/*
*****************************************************************************
  *		XpressNet.h - library for a client XpressNet protocoll
  *		Copyright (c) 2015-2017 Philipp Gahtow  All right reserved.
  *
  **	 Free for Private usage only!	
*****************************************************************************
  * IMPORTANT:
  * 
  * 	Please contact Lenz Inc. for details.
*****************************************************************************
 *  see for changes in XpressNet.h!
*/

// include this library's description file
#include "esp8266-XpressNet.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#endif

#define interval 10500      //interval for Status LED (milliseconds)

XpressNetClass *XpressNetClass::active_object = 0;	//Static

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

XpressNetClass::XpressNetClass()
{
	// initialize this instance's variables 
	Railpower = 0xFF;		//Ausgangs undef.
	XNetclearSendBuf();
	XNetRun = false;	//XNet ist inactive;
	xLokStsclear();		//l�schen aktiver Loks in Slotserver
	ReqLocoAdr = 0;
	ReqLocoAgain = 0;
	ReqFktAdr = 0;
	SlotLast = 0;			
	ReadData = false;		//keine Serial Daten Speichern
}

//******************************************Serial*******************************************
void XpressNetClass::start(byte XAdr, int XControl, bool XControlReverse)  //Initialisierung Serial
{
	ledState = LOW;       // Status LED, used to set the LED
	previousMillis = 0;		//Reset Time Count
	SlotTime = millis();   // will store last time LED was updated
	if (notifyXNetStatus)
		notifyXNetStatus (ledState);

	MY_ADDRESS = XAdr;
	MAX485_CONTROL = XControl;
	MAX485_REVERSE = XControlReverse;
	
	myRequestAck = callByteParity (MY_ADDRESS | 0x00) | 0x100;
	myCallByteInquiry = callByteParity (MY_ADDRESS | 0x40) | 0x100;
	myDirectedOps = callByteParity (MY_ADDRESS | 0x60) | 0x100; 

	//Set up on 62500 Baud
#if defined(__AVR__)
	// LISTEN_MODE 
	if(MAX485_CONTROL >= 0) {
		pinMode(MAX485_CONTROL, OUTPUT);
		digitalWrite (MAX485_CONTROL, MAX485_REVERSE ? HIGH : LOW);
	}

	cli();  //disable interrupts while initializing the USART
	#if defined(SERIAL_PORT)
	 UBRRH = 0;
	 UBRRL = 0x0F;
	 UCSRA = 0;
	 UCSRB = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE) | (1<<UCSZ2);
	 UCSRC = (1<<UCSZ1) | (1<<UCSZ0);
	#elif defined(SERIAL_PORT_0)
	 UBRR0H = 0;
	 UBRR0L = 0x0F;
	 UCSR0A = 0;
	 UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (1<<UCSZ02);
	 UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
	 #elif defined(SERIAL_PORT_1)
	 UBRR1H = 0;
	 UBRR1L = 0x0F;
	 UCSR1A = 0;
	 UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1) | (1<<UCSZ12);
	 UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);
	#endif
	sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed 
	/*
	 *  Enable reception (RXEN = 1).
	 *  Enable transmission (TXEN0 = 1). 
	 *	Enable Receive Interrupt (RXCIE = 1).
	 *  Set 8-bit character mode (UCSZ00, UCSZ01, and UCSZ02 together control this, 
	 *  But UCSZ00, UCSZ01 are in Register UCSR0C).
	 */
#elif defined(ARDUINO_ESP8266_ESP01)
	// start RS485 9 bit interface on 62500 baud rate
	rs485.setup(XNetRS485_RX, XNetRS485_TX);
	rs485.begin(62500, 9);
	if(MAX485_CONTROL >= 0) {
		rs485.setTransmitEnablePin(MAX485_CONTROL, MAX485_REVERSE);
	}
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
	// start RS485 9 bit interface on 62500 baud rate
	rs485.setup(XNetRS485_RX, XNetRS485_TX);
	rs485.begin(62500, 9);
	if(MAX485_CONTROL >= 0) {
		rs485.setTransmitEnablePin(MAX485_CONTROL, MAX485_REVERSE);
	}
#endif
	active_object = this;		//hold Object to call it back in ISR
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

//*******************************************************************************************
//Daten ermitteln und Auswerten
void XpressNetClass::receive(void) 
{
#if defined(ARDUINO_ESP8266_ESP01)
	XNetget();
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
	XNetget();
#endif
	/*
	XNetMsg[XNetlength] = 0x00;
	XNetMsg[XNetmsg] = 0x00;
	XNetMsg[XNetcommand] = 0x00;	savedData[1]
	XNetMsg[XNetdata1] = 0x00;	savedData[2]
	*/
  unsigned long currentMillis = millis();	//aktuelle Zeit setzten

  if (XNetMsg[XNetmsg] != 0x00 && ReadData == false) { 	//Serial Daten dekodieren
//	  previousMillis = millis();   // will store last time LED was updated
	  //Daten, setzte LED = ON!
	  if (ledState == LOW) {	//LED -> aktivieren!
		ledState = HIGH;
		if (notifyXNetStatus)
			notifyXNetStatus (ledState);
	  }
	  if (XNetMsg[XNetmsg] == GENERAL_BROADCAST) {
		  if (XNetMsg[XNetlength] == 4 && XNetMsg[XNetcom] == 0x61) {
			  if ((XNetMsg[XNetdata1] == 0x01) && (XNetMsg[XNetdata2] == 0x60)) {
				// Normal Operation Resumed
				Railpower = csNormal;
				if (notifyXNetPower)
					notifyXNetPower(Railpower);
			  } 
			  else if ((XNetMsg[XNetdata1] == 0x00) && (XNetMsg[XNetdata2] == 0x61)) {
				// Track power off
		        Railpower = csTrackVoltageOff; 
				if (notifyXNetPower)
					notifyXNetPower(Railpower);
			  }
			  else if ((XNetMsg[XNetdata1] == 0x08)) {
				  // Track Short
				  Railpower = csShortCircuit;
				  if (notifyXNetPower) {
					  notifyXNetPower(csTrackVoltageOff);
					  notifyXNetPower(Railpower);
				  }
			  }
			  else if ((XNetMsg[XNetdata1] == 0x02) && (XNetMsg[XNetdata2] == 0x63)) {
				// Service Mode Entry
			    Railpower = csServiceMode;
				if (notifyXNetPower)
					notifyXNetPower(Railpower);
		      }
		  } 
		  else if (XNetMsg[XNetcom] == 0x81) {
			if ((XNetMsg[XNetdata1] == 0x00) && (XNetMsg[XNetdata2] == 0x81)) {
				//Emergency Stop
				Railpower = csEmergencyStop;
				if (notifyXNetPower)
					notifyXNetPower(Railpower);
		     }
		  }
		  else if (XNetMsg[XNetlength] == 8 && XNetMsg[XNetcom] == 0x05 && XNetMsg[XNetdata1] == 0xF1) {
			  //DCC FAST CLOCK set request
			  /* 0x05 0xF1 TCODE1 TCODE2 TCODE3 TCODE4 [XOR]
				00mmmmmm	TCODE1, mmmmmm = denotes minutes, range 0...59.
				100HHHHH	TCODE2, HHHHH = denotes hours, range 0...23.
				01000www	TCODE3, www = denotes day of week, 0=monday, 1=tuesday, a.s.o.
				110fffff	TCODE4, fffff = denotes speed ratio, range 0..31. (0=stopped)
				*/
		  }
	}
	//else if (XNetMsg[XNetmsg] == myDirectedOps && XNetMsg[XNetlength] >= 3) {
		//change by Andr� Schenk
	else if (XNetMsg[XNetlength] >= 3) {	
//		Serial.print("RX: ");
//		Serial.print(XNetMsg[XNetcom], HEX);
//		Serial.print("-");
//		Serial.print(XNetMsg[XNetdata1], HEX);
//		Serial.print(" ");
//		Serial.print(XNetMsg[XNetdata2], HEX);
//		Serial.print(" ");
//		Serial.print(XNetMsg[XNetdata3], HEX);
//		Serial.print(" ");
//		Serial.print(XNetMsg[XNetdata4], HEX);
//		Serial.print("..");

		switch (XNetMsg[XNetcom]) {
		//add by Norberto Redondo Melchor:	
		case 0x52:	// Some other device asked for an accessory change
          //if (XNetMsg[XNetlength] >= 3) {
			  //change by Andr� Schenk
			if (XNetMsg[XNetdata2] >= 0x80) {  
            // Pos = 0000A00P A = turnout output (active 0/inactive 1); P = Turn v1 or --0
            byte A_bit = (XNetMsg[XNetdata2] >> 3) & B0001;
            if (!A_bit) { // Accessory activation request
              unsigned int Adr = (XNetMsg[XNetdata1] << 2) | ((XNetMsg[XNetdata2] & B0110) >> 1);  // Dir afectada
              byte Pos = (XNetMsg[XNetdata2] & B0001) + 1;
              notifyTrnt(highByte(Adr), lowByte(Adr), Pos);
              //digitalWrite(6, HIGH); // Nice to see some blink
            }
            else { // Accessory deactivation request
              //digitalWrite(6, LOW); // Just a blink
            }
          }
		break;
		case 0x62:
			if (XNetMsg[XNetdata1] == 0x21 && XNetMsg[XNetlength] >= 4) {    //Sw Version 2.3
				// old version - version 1 and version 2 response.
	        }
			else if (XNetMsg[XNetdata1] == 0x22 && XNetMsg[XNetlength] >= 5) {
				if (XNetRun == false) {	//Softwareversion anfragen
						unsigned char commandVersionSequence[] = {0x21, 0x21, 0x00};
						XNetSendadd (commandVersionSequence, 3);
						XNetRun = true;
				}
				Railpower = csNormal;
				if (XNetMsg[XNetdata2] != 0) {
					// is track power turned off?
					if ((XNetMsg[XNetdata2] & 0x01) == 0x01) { Railpower = csEmergencyStop; } //Bit 0: wenn 1, Anlage in Nothalt
		            // is it in emergency stop?
					if ((XNetMsg[XNetdata2] & 0x02) == 0x02) { Railpower = csTrackVoltageOff; }  //Bit 1: wenn 1, Anlage in Notaus
					// in service mode?
					if ((XNetMsg[XNetdata2] & 0x08) == 0x08) {Railpower = csServiceMode;}   //Bit 3: wenn 1, dann Programmiermode aktiv          
					// in powerup mode - wait until complete
					if ((XNetMsg[XNetdata2] & 0x40) == 0x40) {
						// put us in a state where we do the status request again...
						XNetRun = false;
					}
				}
				if (notifyXNetPower)
					notifyXNetPower(Railpower);
			}
		break;
		case 0x61:
		  if (XNetMsg[XNetlength] >= 4) {
			if (XNetMsg[XNetdata1] == 0x13) {
				//Programmierinfo �Daten nicht gefunden�
				if (notifyCVInfo)
					notifyCVInfo(0x02);
			}
			if (XNetMsg[XNetdata1] == 0x1F) {
				//Programmierinfo �Zentrale Busy�
				if (notifyCVInfo)
					notifyCVInfo(0x01);
			}
			if (XNetMsg[XNetdata1] == 0x11) {
				//Programmierinfo �Zentrale Bereit�
				if (notifyCVInfo)
					notifyCVInfo(0x00);
			}
			if (XNetMsg[XNetdata1] == 0x12) {
				//Programmierinfo �short-circuit�
				if (notifyCVInfo)
					notifyCVInfo(0x03);
			}
			if (XNetMsg[XNetdata1] == 0x80) {
				//Transfer Error
				if (notifyCVInfo)
					notifyCVInfo(0xE1);
			}
			if (XNetMsg[XNetdata1] == 0x82) {
				//Befehl nicht vorhanden R�ckmeldung
			}
		  }
		break;	
		case 0x63:
			//Softwareversion Zentrale
			if ((XNetMsg[XNetdata1] == 0x21) && (XNetMsg[XNetlength] >= 5)) {
				if (notifyXNetVer)
					notifyXNetVer(XNetMsg[XNetdata2], XNetMsg[XNetdata3]);
			}
			//Programmierinfo �Daten 3-Byte-Format� & �Daten 4-Byte-Format�
			if ((XNetMsg[XNetdata1] == 0x10 || XNetMsg[XNetdata1] == 0x14) && XNetMsg[XNetlength] >= 5) {
				byte cvAdr = XNetMsg[XNetdata2];
				byte cvData = XNetMsg[XNetdata3];
				if (notifyCVResult)
					notifyCVResult(cvAdr, cvData);
			}
		break;
		case 0xE4:	//Antwort der abgefragen Lok
			if (XNetMsg[XNetlength] >= 7 && ReqLocoAdr != 0) {
				byte Adr_MSB = highByte(ReqLocoAdr);
				byte Adr_LSB = lowByte(ReqLocoAdr);
				ReqLocoAdr = 0;
				uint8_t Steps = XNetMsg[XNetdata1];		//0000 BFFF - B=Busy; F=Fahrstufen	
				bitWrite(Steps, 3, 0);	//Busy bit l�schen
				if (Steps == B100)
					Steps = B11;
				boolean Busy = false;
				if (bitRead(XNetMsg[XNetdata1], 3) == 1)
					Busy = true;
				uint8_t Speed = XNetMsg[XNetdata2];		//RVVV VVVV - R=Richtung; V=Geschwindigkeit
				bitWrite(Speed, 7, 0);	//Richtungs bit l�schen
				uint8_t Direction = false;
				if (bitRead(XNetMsg[XNetdata2], 7) == 1)
					Direction = true;
				uint8_t F0 = XNetMsg[XNetdata3];	//0 0 0 F0 F4 F3 F2 F1
				uint8_t F1 = XNetMsg[XNetdata4];	//F12 F11 F10 F9 F8 F7 F6 F5
				
				byte BSteps = Steps;
				if (Busy)
					bitWrite(BSteps, 3, 1);
				byte funcsts = F0;	//FktSts = Chg-F, X, Dir, F0, F4, F3, F2, F1
				bitWrite(funcsts, 5, Direction);	//Direction hinzuf�gen
				
				bool chg = xLokStsadd (Adr_MSB, Adr_LSB, BSteps, Speed, funcsts);	//Eintrag in SlotServer
				chg = chg | xLokStsFunc1 (Adr_MSB, Adr_LSB, F1);
				if (chg == true) 			//�nderungen am Zustand?
					getLocoStateFull(Adr_MSB, Adr_LSB, true);
				
				if (Speed == 0) { //Lok auf Besetzt schalten
					setLocoHalt (Adr_MSB, Adr_LSB);//Sende Lok HALT um Busy zu erzeugen!
				}
			}
		break;
		case 0xE3:	//Antwort abgefrage Funktionen F13-F28
			if (XNetMsg[XNetdata1] == 0x52 && XNetMsg[XNetlength] >= 6 && ReqFktAdr != 0) {	//Funktionszustadn F13 bis F28
				byte Adr_MSB = highByte(ReqFktAdr);
				byte Adr_LSB = lowByte(ReqFktAdr);
				ReqFktAdr = 0;
				byte F2 = XNetMsg[XNetdata2];	//F2 = F20 F19 F18 F17 F16 F15 F14 F13
				byte F3 = XNetMsg[XNetdata3];	//F3 = F28 F27 F26 F25 F24 F23 F22 F21
				if (xLokStsFunc23 (Adr_MSB, Adr_LSB, F2, F3) == true) {	//�nderungen am Zustand?
					if (notifyLokFunc)
						notifyLokFunc(Adr_MSB, Adr_LSB, F2, F3 );
					getLocoStateFull(Adr_MSB, Adr_LSB, true);
				}
			}
			if (XNetMsg[XNetdata1] == 0x40 && XNetMsg[XNetlength] >= 6) { 	// Locomotive is being operated by another device
				XLokStsSetBusy (XNetMsg[XNetdata2], XNetMsg[XNetdata3]);
			}
		break;
		case 0xE1:
			if (XNetMsg[XNetlength] >= 3) {
				//Fehlermeldung Lok control
				if (notifyCVInfo)
					notifyCVInfo(0xE1);
			}
		break;
		case 0x42:	//Antwort Schaltinformation
			if (XNetMsg[XNetlength] >= 4) {
				int Adr = XNetMsg[XNetdata1] * 4;
				byte nibble = bitRead(XNetMsg[XNetdata2], 4);
				byte Pos1 = XNetMsg[XNetdata2] & B11;
				byte Pos2 = (XNetMsg[XNetdata2] >> 2) & B11;
				if (nibble == 1)
					Adr = Adr + 2;
				if (notifyTrnt)
					notifyTrnt(highByte(Adr), lowByte(Adr), Pos1);
				if (notifyTrnt)
					notifyTrnt(highByte(Adr+1), lowByte(Adr+1), Pos2);
			}
		break;
		case 0xA3:	// Locomotive is being operated by another device
			if (XNetMsg[XNetlength] >= 4) {
				if (notifyXNetPower)
					notifyXNetPower(XNetMsg[XNetdata1]);
			}
		break;
		}	//switch myDirectedOps ENDE
	}
//	if (ReadData == false)	//Nachricht komplett empfangen, dann hier l�schen!
		XNetclear();	//alte verarbeitete Nachricht l�schen
  }		//Daten vorhanden ENDE
  else {	//keine Daten empfangen, setzte LED = Blink
	  previousMillis++;
	  if (previousMillis > interval) {			//WARTEN
		  XNetRun = false;	//Keine Zentrale vorhanden
		// save the last time you blinked the LED 
		previousMillis = 0;   
		// if the LED is off turn it on and off (Blink):
		ledState = !ledState;
		if (notifyXNetStatus)
			notifyXNetStatus (ledState);
	  }
  }
  //Slot Server aktualisieren
  if (currentMillis - SlotTime > SlotInterval) {
	  SlotTime = currentMillis;
	  UpdateBusySlot();		//Server Update - Anfrage nach Status�nderungen
  }
}

//--------------------------------------------------------------------------------------------
//Zustand der Gleisversorgung setzten
bool XpressNetClass::setPower(byte Power) 
{
	switch (Power) {	
	case csNormal: {
			unsigned char PowerAn[] = { 0x21, 0x81, 0xA0 };
			return XNetSendadd(PowerAn, 3);
		}
		case csEmergencyStop: {
			unsigned char EmStop[] = { 0x80, 0x80 };
			return XNetSendadd(EmStop, 2);
		}
		case csTrackVoltageOff: {
			unsigned char PowerAus[] = { 0x21, 0x80, 0xA1 };
			return XNetSendadd(PowerAus, 3);
		}
/*		case csShortCircuit:
			return false;
		case csServiceMode:
			return false;	*/
	}
	return false;
}

//--------------------------------------------------------------------------------------------
//Abfrage letzte Meldung �ber Gleispannungszustand
byte XpressNetClass::getPower() 
{
	return Railpower;
}

//--------------------------------------------------------------------------------------------
//Halt Befehl weiterleiten
void XpressNetClass::setHalt()
{
	setPower(csEmergencyStop);
}

//--------------------------------------------------------------------------------------------
//Abfragen der Lokdaten (mit F0 bis F12)
bool XpressNetClass::getLocoInfo (byte Adr_High, byte Adr_Low)	
{
	bool ok = false;
	
	getLocoStateFull(Adr_High, Adr_Low, false);
	
	byte Slot = xLokStsgetSlot(Adr_High, Adr_Low);
	if (xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;		//aktivit�t
		
	if (xLokStsBusy(Slot) == true && ReqLocoAdr == 0)	{		//Besetzt durch anderen XPressNet Handregler
		ReqLocoAdr = word(Adr_High, Adr_Low); //Speichern der gefragen Lok Adresse
		unsigned char getLoco[] = {0xE3, 0x00, Adr_High, Adr_Low, 0x00};
		getXOR(getLoco, 5);
		ok = XNetSendadd (getLoco, 5);
	}
	
	return ok;
}

//--------------------------------------------------------------------------------------------
//Abfragen der Lok Funktionszust�nde F13 bis F28
bool XpressNetClass::getLocoFunc (byte Adr_High, byte Adr_Low)	
{
	if (ReqFktAdr == 0) {
		ReqFktAdr = word(Adr_High, Adr_Low); //Speichern der gefragen Lok Adresse
		unsigned char getLoco[] = {0xE3, 0x09, Adr_High, Adr_Low, 0x00};
		getXOR(getLoco, 5);
		return XNetSendadd (getLoco, 5);
	}
	unsigned char getLoco[] = {0xE3, 0x09, highByte(ReqFktAdr), lowByte(ReqFktAdr), 0x00};
	getXOR(getLoco, 5);
	return XNetSendadd (getLoco, 5);
}

//--------------------------------------------------------------------------------------------
//Lok Stoppen
bool XpressNetClass::setLocoHalt (byte Adr_High, byte Adr_Low)
{
	bool ok = false;
	unsigned char setLocoStop[] = {0x92, Adr_High, Adr_Low, 0x00};
	getXOR(setLocoStop, 4);
	ok = XNetSendadd (setLocoStop, 4);

	byte Slot = xLokStsgetSlot(Adr_High, Adr_Low);
	xLokSts[Slot].speed = 0;	//STOP

	getLocoStateFull(Adr_High, Adr_Low, true);
	return ok;
}

//--------------------------------------------------------------------------------------------
//Lokdaten setzten
bool XpressNetClass::setLocoDrive (byte Adr_High, byte Adr_Low, uint8_t Steps, uint8_t Speed)
{
	bool ok = false;
	unsigned char setLoco[] = {0xE4, 0x10, Adr_High, Adr_Low, Speed, 0x00};
	setLoco[1] |= Steps;
	
	getXOR(setLoco, 6);
	ok = XNetSendadd (setLoco, 6);

	byte Slot = xLokStsgetSlot(Adr_High, Adr_Low);
	xLokSts[Slot].mode = (xLokSts[Slot].mode & B11111100) | Steps;	//Fahrstufen
	xLokSts[Slot].speed = Speed & B01111111;
	bitWrite(xLokSts[Slot].f0, 5, bitRead(Speed, 7));	//Dir

//	getLocoStateFull(Adr_High, Adr_Low, true);	

	//Nutzung protokollieren:
	if (xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;		//aktivit�t
	return ok;
}

//--------------------------------------------------------------------------------------------
//Lokfunktion setzten
bool XpressNetClass::setLocoFunc (byte Adr_High, byte Adr_Low, uint8_t type, uint8_t fkt)
{
	bool ok = false;	//Funktion wurde nicht gesetzt!
	bool fktbit = 0;	//neue zu �ndernde fkt bit
	if (type == 1)	//ein
		fktbit = 1;
	byte Slot = xLokStsgetSlot(Adr_High, Adr_Low);
	//zu �nderndes bit bestimmen und neu setzten:
	if (fkt <= 4) {
		byte func = xLokSts[Slot].f0 & B00011111;	//letztes Zustand der Funktionen 000 F0 F4..F1
		if (type == 2) { //um
			if (fkt == 0)
				fktbit = !(bitRead(func, 4));
			else fktbit = !(bitRead(func, fkt-1));
		}
		if (fkt == 0)
			bitWrite(func, 4, fktbit);
		else bitWrite(func, fkt-1, fktbit);
		//Daten �ber XNet senden:
		unsigned char setLocoFunc[] = {0xE4, 0x20, Adr_High, Adr_Low, func, 0x00};	//Gruppe1 = 0 0 0 F0 F4 F3 F2 F1
		getXOR(setLocoFunc, 6);
		ok = XNetSendadd (setLocoFunc, 6);
		//Slot anpassen:
		if (fkt == 0)
			bitWrite(xLokSts[Slot].f0, 4, fktbit);
		else bitWrite(xLokSts[Slot].f0, fkt-1, fktbit);
	}
	else if ((fkt >= 5) && (fkt <= 8)) {
		byte funcG2 = xLokSts[Slot].f1 & 0x0F;	//letztes Zustand der Funktionen 0000 F8..F5
		if (type == 2) //um
			fktbit = !(bitRead(funcG2, fkt-5));
		bitWrite(funcG2, fkt-5, fktbit);
		//Daten �ber XNet senden:
		unsigned char setLocoFunc[] = {0xE4, 0x21, Adr_High, Adr_Low, funcG2, 0x00};	//Gruppe2 = 0 0 0 0 F8 F7 F6 F5
		getXOR(setLocoFunc, 6);
		ok = XNetSendadd (setLocoFunc, 6);
		//Slot anpassen:
		bitWrite(xLokSts[Slot].f1, fkt-5, fktbit);
	}
	else if ((fkt >= 9) && (fkt <= 12)) {
		byte funcG3 = xLokSts[Slot].f1 >> 4;	//letztes Zustand der Funktionen 0000 F12..F9
		if (type == 2) //um
			fktbit = !(bitRead(funcG3, fkt-9));
		bitWrite(funcG3, fkt-9, fktbit);
		//Daten �ber XNet senden:
		unsigned char setLocoFunc[] = {0xE4, 0x22, Adr_High, Adr_Low, funcG3, 0x00};	//Gruppe3 = 0 0 0 0 F12 F11 F10 F9
		getXOR(setLocoFunc, 6);
		ok = XNetSendadd (setLocoFunc, 6);
		//Slot anpassen:
		bitWrite(xLokSts[Slot].f1, fkt-9+4, fktbit);
	}
	else if ((fkt >= 13) && (fkt <= 20)) {
		byte funcG4 = xLokSts[Slot].f2;
		if (type == 2) //um
			fktbit = !(bitRead(funcG4, fkt-13));
		bitWrite(funcG4, fkt-13, fktbit);
		//Daten �ber XNet senden:
		//unsigned char setLocoFunc[] = {0xE4, 0x23, Adr_High, Adr_Low, funcG4, 0x00};	//Gruppe4 = F20 F19 F18 F17 F16 F15 F14 F13
		unsigned char setLocoFunc[] = {0xE4, 0xF3, Adr_High, Adr_Low, funcG4, 0x00};	//Gruppe4 = F20 F19 F18 F17 F16 F15 F14 F13
		//0xF3 = undocumented command is used when a mulitMAUS is controlling functions f20..f13. 
		getXOR(setLocoFunc, 6);
		ok = XNetSendadd (setLocoFunc, 6);
		//Slot anpassen:
		bitWrite(xLokSts[Slot].f2, (fkt-13), fktbit);
	}
	else if ((fkt >= 21) && (fkt <= 28)) {
		byte funcG5 = xLokSts[Slot].f3;
		if (type == 2) //um
			fktbit = !(bitRead(funcG5, fkt-21));
		bitWrite(funcG5, fkt-21, fktbit);
		//Daten �ber XNet senden:
		unsigned char setLocoFunc[] = {0xE4, 0x28, Adr_High, Adr_Low, funcG5, 0x00};	//Gruppe5 = F28 F27 F26 F25 F24 F23 F22 F21
		getXOR(setLocoFunc, 6);
		ok = XNetSendadd (setLocoFunc, 6);
		//Slot anpassen:
		bitWrite(xLokSts[Slot].f3, (fkt-21), fktbit);
	}
	getLocoStateFull(Adr_High, Adr_Low, true);	//Alle aktiven Ger�te Senden!
	return ok;
}

//--------------------------------------------------------------------------------------------
//Gibt aktuellen Lokstatus an Anfragenden Zur�ck
void XpressNetClass::getLocoStateFull (byte Adr_High, byte Adr_Low, bool bc) 
{
	byte Slot = xLokStsgetSlot(Adr_High, Adr_Low);
	byte Busy = bitRead(xLokSts[Slot].mode, 3);
	byte Dir = bitRead(xLokSts[Slot].f0, 5);
	byte F0 = xLokSts[Slot].f0 & B00011111;
	byte F1 = xLokSts[Slot].f1;
	byte F2 = xLokSts[Slot].f2;
	byte F3 = xLokSts[Slot].f3;
		if (notifyLokAll)
			notifyLokAll(Adr_High, Adr_Low, Busy, xLokSts[Slot].mode & B11, xLokSts[Slot].speed, Dir, F0, F1, F2, F3, bc);
	//Nutzung protokollieren:
	if (xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;		//aktivit�t
}

//--------------------------------------------------------------------------------------------
//Ermitteln der Schaltstellung einer Weiche
bool XpressNetClass::getTrntInfo (byte FAdr_High, byte FAdr_Low)
{
	int Adr = word(FAdr_High, FAdr_Low);
	byte nibble = 0;	// 0 = Weiche 0 und 1; 1 = Weiche 2 und 3
	if ((Adr & 0x03) >= 2)
		nibble = 1;
	unsigned char getTrntPos[] = {0x42, 0x00, 0x80, 0x00};
	getTrntPos[1] = Adr >> 2;
	getTrntPos[2] += nibble;
	getXOR(getTrntPos, 4);
	return XNetSendadd (getTrntPos, 4);
}

//--------------------------------------------------------------------------------------------
//Schalten einer Weiche
bool XpressNetClass::setTrntPos (byte FAdr_High, byte FAdr_Low, byte Pos)
//Pos = 0000A00P	A=Weichenausgang (Aktive/Inaktive); P=Weiche nach links oder nach rechts 
{
	int Adr = word(FAdr_High, FAdr_Low);
	byte AdrL = ((Pos & 0x0F) | B110) & (((Adr & 0x03) << 1) | B1001); //1000ABBP -> A00P = Pos | BB = Adr & 0x03 (LSB Weichenadr.)

	Adr = Adr >> 2;
	bitWrite(AdrL, 7, 1);
	unsigned char setTrnt[] = {0x52, 0x00, AdrL, 0x00};	//old: 0x52, Adr, AdrL, 0x00
	//setTrnt[1] =  (Adr >> 2) & 0xFF;
	//change by Andr� Schenk:
	setTrnt[1] = Adr;
	getXOR(setTrnt, 4);

	//getTrntInfo(FAdr_High, FAdr_Low);  //Schaltstellung abfragen
	if (notifyTrnt)
		notifyTrnt(FAdr_High, FAdr_Low, (Pos & B1) + 1);

	return XNetSendadd (setTrnt, 4);
}

//--------------------------------------------------------------------------------------------
//CV-Mode CV Lesen
void XpressNetClass::readCVMode (byte CV)		
{
	unsigned char cvRead[] = {0x22, 0x15, CV, 0x00};
	getXOR(cvRead, 4);
	XNetSendadd (cvRead, 4);
	getresultCV(); //Programmierergebnis anfordern
}

//--------------------------------------------------------------------------------------------
//Schreiben einer CV im CV-Mode
void XpressNetClass::writeCVMode (byte CV, byte Data)
{
	unsigned char cvWrite[] = {0x23, 0x16, CV, Data, 0x00};
	getXOR(cvWrite, 5);
	XNetSendadd (cvWrite, 5);
	//getresultCV(); //Programmierergebnis anfordern

	if (notifyCVResult)
		notifyCVResult(CV, Data);
}

//--------------------------------------------------------------------------------------------
//Programmierergebnis anfordern
void XpressNetClass::getresultCV ()
{
	unsigned char getresult[] = {0x21, 0x10, 0x31};
	XNetSendadd (getresult, 3);
}

// Private Methods ///////////////////////////////////////////////////////////////////////////////////////////////////
// Functions only available to other functions in this library *******************************************************

//--------------------------------------------------------------------------------------------
// calculate the XOR
void XpressNetClass::getXOR (unsigned char *data, byte length) {
	byte XOR = 0x00;
	for (int i = 0; i < (length-1); i++) {
        XOR = XOR ^ *data;
		data++;
    }
	*data = XOR;
}

//--------------------------------------------------------------------------------------------
// calculate the parity bit in the call byte for this guy
byte XpressNetClass::callByteParity (byte me) {
 byte parity = (1==0);
 byte vv;
 me &= 0x7f;
 vv = me;

 while (vv) {
       parity = !parity;
       vv &= (vv-1);
 }
 if (parity) me |= 0x80;
 return me;
}

//--------------------------------------------------------------------------------------------
int XpressNetClass::USART_Receive(void)
{
#if defined(__AVR__)
	unsigned char status, resh, resl;
	// Wait for data to be received
#if defined(SERIAL_PORT)
	status = UCSRA;
	while (!(status & (1 << RXC))) { return -1; }//status = UCSRA;}

	// Get status and 9th bit, then data
	resh = UCSRB;
	resl = UDR;

	// If error, return -1 
	if (status & ((1 << FE) | (1 << DOR) | (1 << PE))) { return -1; }

#elif defined(SERIAL_PORT_0)
	status = UCSR0A;
	while (!(status & (1 << RXC0))) { return -1; }//status = UCSR0A;}

	// Get status and 9th bit, then data
	resh = UCSR0B;
	resl = UDR0;

	//If error, return -1 
	if (status & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) { return -1; }

#elif defined(SERIAL_PORT_1)
	status = UCSR1A;
	while (!(status & (1 << RXC1))) { return -1; }//status = UCSR1A;}

	// Get status and 9th bit, then data 
	resh = UCSR1B;
	resl = UDR1;

	// If error, return -1
	if (status & ((1 << FE1) | (1 << DOR1) | (1 << UPE1))) { return -1; }
#endif

	// Filter the 9th bit, then return 
	resh = (resh >> 1) & 0x01;
	return ((resh << 8) | resl);
#elif defined(ARDUINO_ESP8266_ESP01)
	if (rs485.available()) {
		int dat = rs485.read();
		return dat;
	} else {
		return -1;
	}
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
	if (rs485.available()) {
		int dat = rs485.read();
		return dat;
	} else {
		return -1;
	}
#endif
}

//--------------------------------------------------------------------------------------------
void XpressNetClass::USART_Transmit(unsigned char data8) {
#if defined(__AVR__)
 // wait for empty transmit buffer
 #if defined(SERIAL_PORT)
  while (!(UCSRA & (1<<UDRE))) {}
 // put the data into buffer, and send 
 UDR = data8;
 #elif defined(SERIAL_PORT_0)
  while (!(UCSR0A & (1<<UDRE0))) {}
 // put the data into buffer, and send 
 UDR0 = data8;
 #elif defined(SERIAL_PORT_1)
 while (!(UCSR1A & (1<<UDRE1))) {}
 // put the data into buffer, and send
 UDR1 = data8;
 #endif
#elif defined(ARDUINO_ESP8266_ESP01)
	rs485.write(data8);
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
	rs485.write(data8);
#endif
}

//--------------------------------------------------------------------------------------------
//L�schen des letzten gesendeten Befehls
void XpressNetClass::XNetclear() 
{
	XNetMsg[XNetlength] = 0x00;
	XNetMsg[XNetmsg] = 0x00;
	XNetMsg[XNetcom] = 0x00;
	XNetMsg[XNetdata1] = 0x00;
	XNetMsg[XNetdata2] = 0x00;
	XNetMsg[XNetdata3] = 0x00;
	XNetMsg[XNetdata4] = 0x00;
	XNetMsg[XNetdata5] = 0x00;
}

//--------------------------------------------------------------------------------------------
//Interrupt routine for reading via Serial
#if defined(__AVR__)
#if defined(SERIAL_PORT)
ISR(USART_RX_vect)  {
	XpressNetClass::handle_interrupt();	 //weiterreichen an die Funktion
}

#elif defined(SERIAL_PORT_0)
ISR(USART_RX_vect) {
	XpressNetClass::handle_interrupt();	 //weiterreichen an die Funktion
}

#elif defined(SERIAL_PORT_1)
ISR(USART1_RX_vect) {
	XpressNetClass::handle_interrupt();	 //weiterreichen an die Funktion
}
#endif
#elif defined(ARDUINO_ESP8266_ESP01)
// void RS485Data() {
// 	XpressNetClass::handle_interrupt();
// }
#endif

// Interrupt handling
/* static */
inline void XpressNetClass::handle_interrupt()
{
  if (active_object)
  {
    active_object->XNetget();	//Daten Einlesen und Speichern
  }
}

//--------------------------------------------------------------------------------------------
//Serial einlesen:
void XpressNetClass::XNetget() 
{
  const uint16_t rxdata = USART_Receive();

	if (rxdata != 0xffff) {		//Daten wurden korrekt empfangen?
		if (notifyXNetData)
			notifyXNetData(rxdata);

		// Serial.println(rxdata, BIN);
		previousMillis = 0;		//Reset Time Count
		// This IS a Call Byte
		if (rxdata >= 0x100) {		//Neue Nachricht beginnen
			ReadData = false;		//keine Speichern der Serial Daten
			if (rxdata == myRequestAck) {
				unsigned char requestAckAck[] = {0x20, 0x20};
				XNetsend(requestAckAck, 2);
				//Transfer Error
				if (notifyCVInfo)
					notifyCVInfo(0xE1);
				return;		//Daten wurden verarbeitet
			}
			else if (rxdata == myCallByteInquiry) {
				unsigned char commandStatusSequence[] = {0x21, 0x24, 0x05};
				if (XNetRun == false || Railpower == 0xFF) {
					XNetsend(commandStatusSequence, 3);
				}
				else XNetsend();
				return;		//Daten wurden verarbeitet
			}
			else if (rxdata == GENERAL_BROADCAST || rxdata == myDirectedOps) {	//Datenempfang aktivieren
				XNetclear();	//alte Nachricht l�schen
				ReadData = true;
			}
		}
		//add by Norberto Redondo Melchor:
		else if (rxdata == 0x52) {  // Let's spy on the bus: someone has requested an accessory change
			XNetMsg[XNetmsg] = 0x01;  // Any non-zero value would do
			XNetMsg[XNetlength] = 1;
			ReadData = true;          // Let's record this someone's request
		}
		if (ReadData == true) {   //Data is for our own address
			// Serial.println(rxdata, BIN);
			XNetMsg[XNetlength]++;      //Let's make room for it...
			XNetMsg[XNetMsg[XNetlength]] = rxdata;  //...and store it
		}
	}
}

//--------------------------------------------------------------------------------------------
void XpressNetClass::XNetclearSendBuf()		//Buffer leeren
{
	for (byte i = 0; i < XSendMax; i++) {
		XNetSend[i].length = 0x00;			//L�nge zur�cksetzten
		for (byte j = 0; j < XSendMaxData; j++) {
			XNetSend[i].data[j] = 0x00;		//Daten l�schen
		}
	}
}

//--------------------------------------------------------------------------------------------
boolean XpressNetClass::XNetSendadd(unsigned char *dataString, byte byteCount) 
{
	for (byte i = 0; i < XSendMax; i++) {
		if (XNetSend[i].length == 0) {	//Daten hier Eintragen:
			XNetSend[i].length = byteCount;	 //Datenlaenge
			for (byte b = 0; b < byteCount; b++) {
				XNetSend[i].data[b] = *dataString;
				dataString++;
			}
			return true;	//leeren Platz gefunden -> ENDE
		}
	}
	return false;	//Kein Platz im Sendbuffer frei!
}

//--------------------------------------------------------------------------------------------
//Byte via Serial senden
void XpressNetClass::XNetsend(void)
{	
	if (XNetSend[0].length != 0) { // && XNetSend[0].length < XSendMaxData) {
		if (XNetSend[0].data[0] != 0)
			XNetsend(XNetSend[0].data,XNetSend[0].length);
		for (int i = 0; i < (XSendMax-1); i++) {
			XNetSend[i].length = XNetSend[i+1].length;
			for (int j = 0; j < XSendMaxData; j++) {
				XNetSend[i].data[j] = XNetSend[i+1].data[j];		//Daten kopieren
			}
		}
		//letzten Leeren
		XNetSend[XSendMax-1].length = 0x00;
		for (int j = 0; j < XSendMaxData; j++) {
			XNetSend[XSendMax-1].data[j] = 0x00;		//Daten l�schen
		}
	}
	else XNetSend[0].length = 0;
}

//--------------------------------------------------------------------------------------------
// send along a bunch of bytes to the Command Station
void XpressNetClass::XNetsend(unsigned char *dataString, byte byteCount) {
	 unsigned int i;
#if defined(__AVR__)
	 if(MAX485_CONTROL >= 0) {
		 digitalWrite (MAX485_CONTROL, MAX485_REVERSE ? LOW : HIGH);
	 }
#endif
//   delayMicroseconds(3);
   for (i=0; i< byteCount; i++) {
     USART_Transmit (*dataString);
     dataString ++;
	}
#if defined(__AVR__)
	 WAIT_FOR_XMIT_COMPLETE;
	 if(MAX485_CONTROL >= 0) {
		 digitalWrite (MAX485_CONTROL, MAX485_REVERSE ? HIGH : LOW);
	 }
#endif
   
} 


/*
***************************************** SLOTSERVER ****************************************
	uint8_t low;		// A7, A6, A5, A4, A3, A2, A1, A0
	uint8_t high;		//X, X, A13, A12, A11, A10, A9, A8
	uint8_t speed;		//Speed 0..127 (0x00 - 0x7F)
	uint8_t f0;		//0, 0, Dir, F0, F4, F3, F2, F1
	uint8_t f1;
	uint8_t func3;
	uint8_t func4;
	uint8_t state;	//Zahl der Zugriffe
*/

//--------------------------------------------------------------------------------------------
void XpressNetClass::UpdateBusySlot(void)	//Fragt Zentrale nach aktuellen Zust�nden
{
/*
	if (ReqLocoAdr == 0) {
		if (xLokStsIsEmpty(SlotLast) == false && xLokSts[SlotLast].state > 0 && xLokStsBusy(SlotLast) == true) { 
			byte Adr_High = xLokSts[SlotLast].high & 0x3F;
			byte Adr_Low = xLokSts[SlotLast].low; 
			ReqLocoAdr = word(Adr_High, Adr_Low); //Speichern der gefragen Lok Adresse
			unsigned char getLoco[] = {0xE3, 0x00, Adr_High, Adr_Low, 0x00};
			getXOR(getLoco, 5);
			XNetSendadd (getLoco, 5);
//			if (bitRead(xLokSts[SlotLast].mode, 3) == 1)	//Slot BUSY?
				getLocoFunc (Adr_High, Adr_Low);	//F13 bis F28 abfragen
		}
		int Slot = SlotLast;
		SlotLast = getNextSlot(SlotLast);	//n�chste Lok holen
		while (SlotLast != Slot) {
			if (xLokStsBusy(SlotLast) == true) {
				Slot = SlotLast;
				break;
			}
			SlotLast = getNextSlot(SlotLast);	//n�chste Lok holen
		}
		
	}
	else
*/	
	if (ReqLocoAdr != 0) {
		ReqLocoAgain++;
		if (ReqLocoAgain > 9) {
			unsigned char getLoco[] = {0xE3, 0x00, highByte(ReqLocoAdr), lowByte(ReqLocoAdr), 0x00};
			getXOR(getLoco, 5);
			XNetSendadd (getLoco, 5);
			ReqLocoAgain = 0;
		}
	}
/*
	//Nichtnutzung von Slots erfassen:
	for (int i = 0; i < SlotMax; i++) {
		if (xLokSts[i].state > 0)
			xLokSts[i].state--;
		if (xLokSts[i].state > 0)
			xLokSts[i].state--;
	}
*/
}

//--------------------------------------------------------------------------------------------
void XpressNetClass::xLokStsclear (void)	//l�scht alle Slots
{
	for (int i = 0; i < SlotMax; i++) {
		xLokSts[i].low = 0xFF;
		xLokSts[i].high = 0xFF;
		xLokSts[i].mode = 0xFF;
		xLokSts[i].speed = 0xFF;
		xLokSts[i].f0 = 0xFF;
		xLokSts[i].f1 = 0xFF;
		xLokSts[i].f2 = 0xFF;
		xLokSts[i].f3 = 0xFF;
		xLokSts[i].state = 0x00;
	}
}

//--------------------------------------------------------------------------------------------
bool XpressNetClass::xLokStsadd (byte MSB, byte LSB, byte Mode, byte Speed, byte FktSts)	//Eintragen �nderung / neuer Slot XLok
{
	bool change = false;
	byte Slot = xLokStsgetSlot(MSB, LSB);
	if (xLokSts[Slot].mode != Mode) {	//Busy & Fahrstufe (keine 14 Fahrstufen!)
		xLokSts[Slot].mode = Mode;
		change = true;
	}
	if (xLokSts[Slot].speed != Speed) {
		xLokSts[Slot].speed = Speed;
		change = true;
	}
	//FktSts = X, X, Dir, F0, F4, F3, F2, F1
	if (xLokSts[Slot].f0 != FktSts) {
		xLokSts[Slot].f0 = FktSts;
		change = true;
	}
	if (change == true && xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;
	return change;
}

//--------------------------------------------------------------------------------------------
bool XpressNetClass::xLokStsFunc0 (byte MSB, byte LSB, byte Func)	//Eintragen �nderung / neuer Slot XFunc
{
	bool change = false;
	byte Slot = xLokStsgetSlot(MSB, LSB);
	if ((xLokSts[Slot].f0 & B00011111) != Func) {
		xLokSts[Slot].f0 = Func | (xLokSts[Slot].f0 & B00100000);	//Dir anh�ngen!
		change = true;
	}
	if (change == true && xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;
	return change;
}

//--------------------------------------------------------------------------------------------
bool XpressNetClass::xLokStsFunc1 (byte MSB, byte LSB, byte Func1)	//Eintragen �nderung / neuer Slot XFunc1
{
	bool change = false;
	byte Slot = xLokStsgetSlot(MSB, LSB);
	if (xLokSts[Slot].f1 != Func1) {
		xLokSts[Slot].f1 = Func1;
		change = true;
	}
	if (change == true && xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;
	return change;
}

//--------------------------------------------------------------------------------------------
bool XpressNetClass::xLokStsFunc23 (byte MSB, byte LSB, byte Func2, byte Func3)	//Eintragen �nderung / neuer Slot
{
	bool change = false;
	byte Slot = xLokStsgetSlot(MSB, LSB);
	if (xLokSts[Slot].f2 != Func2) {
		xLokSts[Slot].f2 = Func2;
		change = true;
	}
	if (xLokSts[Slot].f3 != Func3) {
		xLokSts[Slot].f3 = Func3;
		change = true;
	}
	if (change == true && xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;
	return change;
}

bool XpressNetClass::xLokStsBusy (byte Slot) {
	bool Busy = false;
	if (bitRead(xLokSts[Slot].mode, 3) == 1)
		Busy = true;
	return Busy;
}

void XpressNetClass::XLokStsSetBusy (byte MSB, byte LSB) {
	byte Slot = xLokStsgetSlot(MSB, LSB);
	bitWrite(xLokSts[Slot].mode, 3, 1);
	if (xLokSts[Slot].state < 0xFF)
		xLokSts[Slot].state++;
}

//--------------------------------------------------------------------------------------------
byte XpressNetClass::xLokStsgetSlot (byte MSB, byte LSB)		//gibt Slot f�r Adresse zur�ck / erzeugt neuen Slot (0..126)
{
	byte Slot = 0x00;	//kein Slot gefunden!
	for (int i = 0; i < SlotMax; i++) {
		if ((xLokSts[i].low == LSB && xLokSts[i].high == MSB) || xLokStsIsEmpty(i)) {
			Slot = i;			//Slot merken
			if (xLokStsIsEmpty(Slot)) 	//neuer freier Slot - Lok eintragen
				xLokStsSetNew(Slot, MSB, LSB);	//Eintragen
			return Slot;
		}
	}
	//kein Slot mehr vorhanden!
	byte zugriff = 0xFF;
	for (int i = 0; i < SlotMax; i++) {
		if (xLokSts[i].state < zugriff) {
			Slot = i;
			zugriff = xLokSts[i].state;
		}
	}
	xLokStsSetNew(Slot, MSB, LSB);	//Eintragen
	return Slot;
}

//--------------------------------------------------------------------------------------------
int XpressNetClass::xLokStsgetAdr (byte Slot)			//gibt Lokadresse des Slot zur�ck, wenn 0x0000 dann keine Lok vorhanden
{
	if (!xLokStsIsEmpty(Slot))
		return word(xLokSts[Slot].high, xLokSts[Slot].low);	//Addresse zur�ckgeben
	return 0x0000;
}

//--------------------------------------------------------------------------------------------
bool XpressNetClass::xLokStsIsEmpty (byte Slot)	//pr�ft ob Datenpacket/Slot leer ist?
{
	if (xLokSts[Slot].low == 0xFF && xLokSts[Slot].high == 0xFF && xLokSts[Slot].speed == 0xFF && xLokSts[Slot].f0 == 0xFF && 
		xLokSts[Slot].f1 == 0xFF && xLokSts[Slot].f2 == 0xFF && xLokSts[Slot].f3 == 0xFF && xLokSts[Slot].state == 0x00)
		return true;
	return false;
}

//--------------------------------------------------------------------------------------------
void XpressNetClass::xLokStsSetNew (byte Slot, byte MSB, byte LSB)	//Neue Lok eintragen mit Adresse
{
	xLokSts[Slot].low = LSB;
	xLokSts[Slot].high = MSB;
	xLokSts[Slot].mode = B1011;	//Busy und 128 Fahrstufen
	xLokSts[Slot].speed = 0x00;
	xLokSts[Slot].f0 = 0x00;
	xLokSts[Slot].f1 = 0x00;
	xLokSts[Slot].f2 = 0x00;
	xLokSts[Slot].f3 = 0x00;
	xLokSts[Slot].state = 0x00;
}

//--------------------------------------------------------------------------------------------
byte XpressNetClass::getNextSlot (byte Slot)	//gibt n�chsten genutzten Slot
{
	byte nextS = Slot;
	for (int i = 0; i < SlotMax; i++) {
		nextS++;	//n�chste Lok
		if (nextS >= SlotMax)
			nextS = 0;	//Beginne von vorne
		if (xLokStsIsEmpty(nextS) == false)
			return nextS;
	}
	return nextS;
}

//--------------------------------------------------------------------------------------------
void XpressNetClass::setFree(byte MSB, byte LSB)		//Lok aus Slot nehmen
{
	byte Slot = xLokStsgetSlot(MSB, LSB);
	xLokSts[Slot].low = 0xFF;
	xLokSts[Slot].high = 0xFF;
	xLokSts[Slot].mode = 0xFF;
	xLokSts[Slot].speed = 0xFF;
	xLokSts[Slot].f0 = 0xFF;
	xLokSts[Slot].f1 = 0xFF;
	xLokSts[Slot].f2 = 0xFF;
	xLokSts[Slot].f3 = 0xFF;
	xLokSts[Slot].state = 0x00;
}