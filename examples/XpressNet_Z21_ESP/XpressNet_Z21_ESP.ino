/*
    Z21 Ethernet Emulation für die App-Steuerung via Smartphone über XpressNet.
    by Philipp Gahtow (c) 2016-2017
    Email: digitalmoba@arcor.de
    
    Version 2.3
    
    Änderungen:
    - Soft Serial Debug Funktion für Arduino MEGA
    - Enc28j60 kompartibel (keine sichere Kommunikation!!!)
    - S88 Rückmelde Bus
    - DHCP  (neu!)
    - Bug Fix für Weichensteuerung by André Schenk
 */
 
//----------------------------------------------------------------------------
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(ARDUINO_ESP8266_ESP01)
#define DEBUG  //For Serial Port Debugging (Arduino Mega and esp8266 only)
#endif  

//#define WEBCONFIG //HTTP Port 80 Website zur Konfiguration
#define DHCP      //Activate to Receive a IP Adress from the DHCP Server, if no DHCP found fix IP Adress vom EEPROM will be load.
//----------------------------------------------------------------------------

#include <EEPROM.h>

#include <esp8266-XpressNet.h> 
XpressNetClass XpressNet;

//For use with Arduino_UIP and Enc28j60
//#define USEENC28
//#include <UIPEthernet.h>

//For use with Standard W5100 Library
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>         // UDP library

#define EES88Moduls 38      //Adresse EEPROM Anzahl der Module für S88
#define EEip 40    //Startddress im EEPROM für die IP
#define EEXNet 45   //Adresse im XNet-Bus

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[6] = {0xFE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 111);

// An EthernetUDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

#if defined(WEBCONFIG)
 // (port 80 is default for HTTP):
 EthernetServer server(80);
#endif

#define localPort 21105      // Z21 local port to listen on
#define XNetTxRxPin 5    //Send/Receive Pin MAX

// #define Z21ResetPin A5  //Reset Pin bei Neustart betätigen um Standard IP zu setzten!

//--------------------------------------------------------------
//S88 Timer frequency is 250kHz for ( /64 prescale from 16MHz )
// #define TIMER_Time 0x50 //je größer desto schneller die Abfrageintervalle
/*
 Der Timer erzeugt den notwendigen Takt für die S88 Schiebeabfragen.
Je nach verwendten Modulen kann der Takt beliebigt in seiner Geschwindigkeit
geändert werden, aber nicht jede Hardware unterstützt ein "fast" Auslesen!
*/
//Pinbelegungen am Dekoder:
//Eingänge:
// #define S88DataPin A0      //S88 Data IN

//Ausgänge:
// #define S88ClkPin A1    //S88 Clock
// #define S88PSPin A2    //S88 PS/LOAD
// #define S88ResetPin A3    //S88 Reset

// uint8_t S88RCount = 0;    //Lesezähler 0-39 Zyklen
// uint8_t S88RMCount = 0;   //Lesezähler Modul-Pin

/*
'0' = keine
's' = Änderungen vorhanden, noch nicht fertig mit Auslesen
'i' = Daten vollständig, senden an PC
*/
// char S88sendon = '0';        //Bit Änderung

// byte S88Module = 0;    //Anzahl der Module - maximal 62 Module à 16 Ports

byte data[62];     //Zustandsspeicher für 62x 8fach Modul

//--------------------------------------------------------------
// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.
byte XNetAddress = 30;    //Adresse im XpressNet
#define XBusVer 0x30      //Version XNet-Bus (default 3.0)

// buffers for receiving and sending data
#define UDP_TX_MAX_SIZE 10
unsigned char packetBuffer[UDP_TX_MAX_SIZE]; //buffer to hold incoming packet, 
//--> UDP_TX_PACKET_MAX_SIZE

#define maxIP 10        //Speichergröße für IP-Adressen
#define ActTimeIP 20    //Aktivhaltung einer IP für (sec./2)
#define interval 2000   //interval at milliseconds

struct TypeActIP {
 byte ip0;    // Byte IP
 byte ip1;    // Byte IP
 byte ip2;    // Byte IP
 byte ip3;    // Byte IP
 byte BCFlag;  //BoadCastFlag 4. Byte Speichern
 byte time;  //Zeit
};
TypeActIP ActIP[maxIP];    //Speicherarray für IPs

long previousMillis = 0;        // will store last time of IP decount updated

#if defined(DEBUG)
  #define Debug Serial 
//  #include <SoftwareSerial.h>
//  SoftwareSerial Debug(0, 1); // RX, TX
#endif

//--------------------------------------------------------------------------------------------
void setup() {
//  #if defined(USEENC28)
//    /* Disable SD card */
//    pinMode(4, OUTPUT);
//    digitalWrite(4, HIGH);
//  #endif  
 
 #if defined(DEBUG)
   Debug.begin(115200); 
   Debug.println("Z21 XpressNet Client");
 #endif
//  pinMode(S88ResetPin, OUTPUT);    //Reset
//  pinMode(S88PSPin, OUTPUT);      //PS/LOAD
//  pinMode(S88ClkPin, OUTPUT);      //Clock
//  digitalWrite(S88ResetPin, LOW);
//  digitalWrite(S88PSPin, LOW);      //init
//  digitalWrite(S88ClkPin, LOW);
//  pinMode(S88DataPin, INPUT_PULLUP);    //Dateneingang

//  pinMode(Z21ResetPin, INPUT_PULLUP);  
 delay(50);
//  if (digitalRead(Z21ResetPin) == LOW || EEPROM.read(EEXNet) > 32) {
//    #if defined(DEBUG)
//      Debug.println("RESET IP"); 
//    #endif  
//    EEPROM.write(EEXNet, XNetAddress);
//    EEPROM.write(EEip, ip[0]);
//    EEPROM.write(EEip+1, ip[1]);
//    EEPROM.write(EEip+2, ip[2]);
//    EEPROM.write(EEip+3, ip[3]);
//  }
//  XNetAddress = EEPROM.read(EEXNet);
//  ip[0] = EEPROM.read(EEip);
//  ip[1] = EEPROM.read(EEip+1);
//  ip[2] = EEPROM.read(EEip+2);
//  ip[3] = EEPROM.read(EEip+3);
 
 #if defined(DHCP)
   #if defined(DEBUG)
    Debug.print("IP via DHCP... ");
   #endif 
  if (WiFi.begin("n3t", "aturiociv") == 0) {    //IP via DHCP
    #if defined(DEBUG)
        Debug.println(F("DHCP fail!")); 
    #endif
    #undef DHCP
  } else {
    byte counter = 0; 
    while (WiFi.status() != WL_CONNECTED && counter < 1000) {
      counter++;
      #if defined(ESP8266_LED)
      if (led) {
        digitalWrite(ESP8266_LED, HIGH);
        led = false;
      } else {
        digitalWrite(ESP8266_LED, LOW);
        led = true;
      }
      #endif
      delay(100);
    }
    //Save IP that receive from DHCP
    ip = WiFi.localIP();
  }
 #endif
//  #if !defined(DHCP)
//  // initialize the Ethernet device not using DHCP:
//  WiFi.begin(mac,ip);  //IP and MAC Festlegung
//  #endif
 
 #if defined(DEBUG)
   Debug.println(ip);
   Debug.print("XAdr: ");
   Debug.println(XNetAddress);
//    Debug.print("S88 Module: ");
//    Debug.println(S88Module);
 #endif
 // start the Webserver:
 #if defined(WEBCONFIG)
   server.begin();    //HTTP Server
 #endif  
 // start the UDP Server
 Udp.begin(localPort);  //UDP Z21 Port

 XpressNet.start(XNetAddress, XNetTxRxPin, true);    //Initialisierung XNet und Send/Receive-PIN

 for (int i = 0; i < maxIP; i++)
   clearIPSlot(i);  //löschen gespeicherter aktiver IP's
 
//  SetupsS88();    //initialize Timer2 for S88

 #if defined(DEBUG)
  Debug.print("RAM: ");
  Debug.println(freeRam());  
 #endif 
}

/*
//--------------------------------------------------------------------------------------------
void notifyXNetVer(uint8_t V, uint8_t ID ) {
}

//--------------------------------------------------------------------------------------------
void notifyXNetStatus(uint8_t LedState ) {
}
*/

//--------------------------------------------------------------------------------------------
void loop() {

 XpressNet.receive();  //Check for XpressNet

 Ethreceive();    //Read Data on UDP Port

 XpressNet.receive();  //Check for XpressNet

 #if defined(WEBCONFIG)
   Webconfig();    //Webserver for Configuration
 #endif  
 
//  notifyS88Data();    //R-Bus geänderte Daten Melden

 //Nicht genutzte IP's aus Speicher löschen
 unsigned long currentMillis = millis();
 if(currentMillis - previousMillis > interval) {
   previousMillis = currentMillis;   
   for (int i = 0; i < maxIP; i++) {
     if (ActIP[i].ip3 != 0) {  //Slot nicht leer?
       if (ActIP[i].time > 0) 
         ActIP[i].time--;    //Zeit herrunterrechnen
       else {
         #if defined(DEBUG)
           Debug.print("Clear IP ");
           Debug.println(ActIP[i].ip3);
         #endif  
         clearIPSlot(i);   //clear IP DATA
       }
     }
   }
 }
}

//--------------------------------------------------------------------------------------------
void clearIPSlots() {
 for (int i = 0; i < maxIP; i++)
   clearIPSlot(i);
}

//--------------------------------------------------------------------------------------------
//Slot mit Nummer "i" löschen
void clearIPSlot(byte i) {
 ActIP[i].ip0 = 0;
 ActIP[i].ip1 = 0;
 ActIP[i].ip2 = 0;
 ActIP[i].ip3 = 0;
 ActIP[i].BCFlag = 0;
 ActIP[i].time = 0;
}

//--------------------------------------------------------------------------------------------
void clearIPSlot(byte ip0, byte ip1, byte ip2, byte ip3) {
 for (byte i = 0; i < maxIP; i++) {
   if (ActIP[i].ip0 == ip0 && ActIP[i].ip1 == ip1 && ActIP[i].ip2 == ip2 && ActIP[i].ip3 == ip3) 
     clearIPSlot(i);
 }
}

//--------------------------------------------------------------------------------------------
byte addIPToSlot (byte ip0, byte ip1, byte ip2, byte ip3, byte BCFlag) {
 byte Slot = maxIP;
 for (int i = 0; i < maxIP; i++) {
   if (ActIP[i].ip0 == ip0 && ActIP[i].ip1 == ip1 && ActIP[i].ip2 == ip2 && ActIP[i].ip3 == ip3) {
     ActIP[i].time = ActTimeIP;
     if (BCFlag != 0)    //Falls BC Flag übertragen wurde diesen hinzufügen!
       ActIP[i].BCFlag = BCFlag;
     return ActIP[i].BCFlag;    //BC Flag 4. Byte Rückmelden
   }
   else if (ActIP[i].time == 0 && Slot == maxIP)
     Slot = i;
 }
 ActIP[Slot].ip0 = ip0;
 ActIP[Slot].ip1 = ip1;
 ActIP[Slot].ip2 = ip2;
 ActIP[Slot].ip3 = ip3;
 ActIP[Slot].time = ActTimeIP;
 notifyXNetPower(XpressNet.getPower());
 return ActIP[Slot].BCFlag;   //BC Flag 4. Byte Rückmelden
}

//--------------------------------------------------------------------------------------------
#if defined(WEBCONFIG)
void Webconfig() {
 EthernetClient client = server.available();
 if (client) {
   String receivedText = String(50);
   // an http request ends with a blank line
   boolean currentLineIsBlank = true;
   while (client.connected()) {
     if (client.available()) {
       char c = client.read();
       if (receivedText.length() < 50) {
         receivedText += c;
       }
       // if you've gotten to the end of the line (received a newline
       // character) and the line is blank, the http request has ended,
       // so you can send a reply
       if (c == '\n' && currentLineIsBlank) {
         // send a standard http response header
         client.println("HTTP/1.1 200 OK");
         client.println("Content-Type: text/html");
         //client.println("Connection: close");  // the connection will be closed after completion of the response
         //client.println("Refresh: 5");  // refresh the page automatically every 5 sec
         client.println();
         //Website:
         client.println("<html><head><title>Z21</title></head><body>");
         client.println("<h1>Z21</h1><br />");
         //----------------------------------------------------------------------------------------------------          
         int firstPos = receivedText.indexOf("?");
         if (firstPos > -1) {
           client.println("-> accept change after RESET!");
           byte lastPos = receivedText.indexOf(" ", firstPos);
           String theText = receivedText.substring(firstPos+3, lastPos); // 10 is the length of "?A="
           byte S88Pos = theText.indexOf("&S88=");
           S88Module = theText.substring(S88Pos+5, theText.length()).toInt();
           byte XNetPos = theText.indexOf("&XNet=");
           XNetAddress = theText.substring(XNetPos+6, S88Pos).toInt();
           byte Aip = theText.indexOf("&B=");
           byte Bip = theText.indexOf("&C=", Aip);
           byte Cip = theText.indexOf("&D=", Bip);
           byte Dip = theText.substring(Cip+3, XNetPos).toInt();
           Cip = theText.substring(Bip+3, Cip).toInt();
           Bip = theText.substring(Aip+3, Bip).toInt();
           Aip = theText.substring(0, Aip).toInt();
           ip[0] = Aip;
           ip[1] = Bip;
           ip[2] = Cip;
           ip[3] = Dip;
           if (EEPROM.read(EES88Moduls) != S88Module) {
             EEPROM.write(EES88Moduls, S88Module);
             SetupS88();
           }
           if (EEPROM.read(EEXNet) != XNetAddress)
             EEPROM.write(EEXNet, XNetAddress);
           if (EEPROM.read(EEip) != Aip)  
             EEPROM.write(EEip, Aip);
           if (EEPROM.read(EEip+1) != Bip)  
             EEPROM.write(EEip+1, Bip);
           if (EEPROM.read(EEip+2) != Cip)  
             EEPROM.write(EEip+2, Cip);
           if (EEPROM.read(EEip+3) != Dip)  
             EEPROM.write(EEip+3, Dip);
         }
         //----------------------------------------------------------------------------------------------------          
         client.print("<form method=get>IP-Adr.: <input type=number min=10 max=254 name=A value=");
         client.println(ip[0]);
         client.print(">.<input type=number min=0 max=254 name=B value=");
         client.println(ip[1]);
         client.print(">.<input type=number min=0 max=254 name=C value=");
         client.println(ip[2]);
         client.print(">.<input type=number min=0 max=254 name=D value=");
         client.println(ip[3]);
         client.print("><br /> XBus Adr.: <input type=number min=1 max=31 name=XNet value=");
         client.print(XNetAddress);
         client.print("><br /> S88 8x Module: <input type=number min=0 max=62 name=S88 value=");
         client.print(S88Module);
         client.println("><br /><br />");
         client.println("<input type=submit></form>");
         client.println("</body></html>");
         break;
       }
       if (c == '\n') 
         currentLineIsBlank = true; // you're starting a new line
       else if (c != '\r') 
         currentLineIsBlank = false; // you've gotten a character on the current line
     }
   }
   client.stop();  // close the connection:
 }
}
#endif

//--------------------------------------------------------------------------------------------
void Ethreceive() {
 int packetSize = Udp.parsePacket();
 if(packetSize > 0) {
   addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3], 0);
   Udp.read(packetBuffer,UDP_TX_MAX_SIZE);  // read the packet into packetBufffer
   // send a reply, to the IP address and port that sent us the packet we received
   int header = (packetBuffer[3]<<8) + packetBuffer[2];
   //    int datalen = (packetBuffer[1]<<8) + packetBuffer[0];
   byte data[16]; 
   //boolean ok = false;
   switch (header) {
   case 0x10:
     #if defined(DEBUG)
       Debug.println("LAN_GET_SERIAL_NUMBER");  
     #endif
     data[0] = 0xF5;  //Seriennummer 32 Bit (little endian)
     data[1] = 0x0A;
     data[2] = 0x00; 
     data[3] = 0x00;
     EthSend (0x08, 0x10, data, false, 0x00);
     break; 
   case 0x1A:
     #if defined(DEBUG)
       Debug.println("LAN_GET_HWINFO"); 
     #endif
     data[0] = 0x01;  //HwType 32 Bit
     data[1] = 0x02;
     data[2] = 0x02; 
     data[3] = 0x00;
     data[4] = 0x20;  //FW Version 32 Bit
     data[5] = 0x01;
     data[6] = 0x00; 
     data[7] = 0x00;
     EthSend (0x0C, 0x1A, data, false, 0x00);
     break;  
   case 0x30:
     #if defined(DEBUG) 
       Debug.println("LAN_LOGOFF"); 
     #endif
     clearIPSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3]);
     //Antwort von Z21: keine
     break; 
     case (0x40):
     switch (packetBuffer[4]) { //X-Header
     case 0x21: 
       switch (packetBuffer[5]) {  //DB0
       case 0x21:
         #if defined(DEBUG) 
           Debug.println("LAN_X_GET_VERSION"); 
         #endif
         data[0] = 0x63;
         data[1] = 0x21;
         data[2] = XBusVer;   //X-Bus Version
         data[3] = 0x12;  //ID der Zentrale
         EthSend (0x09, 0x40, data, true, 0x00);
         break;
       case 0x24:
         data[0] = 0x62;
         data[1] = 0x22;
         data[2] = XpressNet.getPower();
         //Debug.print("LAN_X_GET_STATUS "); 
         //Debug.println(data[2], HEX);
         EthSend (0x08, 0x40, data, true, 0x00);
         break;
       case 0x80:
         #if defined(DEBUG) 
           Debug.println("LAN_X_SET_TRACK_POWER_OFF"); 
         #endif
         XpressNet.setPower(csTrackVoltageOff);
         break;
       case 0x81:
         #if defined(DEBUG) 
           Debug.println("LAN_X_SET_TRACK_POWER_ON"); 
         #endif
         XpressNet.setPower(csNormal);
         break;  
       }
       break;
     case 0x23:
       if (packetBuffer[5] == 0x11) {  //DB0
         #if defined(DEBUG) 
           Debug.println("LAN_X_CV_READ"); 
         #endif
         byte CV_MSB = packetBuffer[6];
         byte CV_LSB = packetBuffer[7];
         XpressNet.readCVMode(CV_LSB+1);
       }
       break;             
     case 0x24:
       if (packetBuffer[5] == 0x12) {  //DB0
         #if defined(DEBUG) 
           Debug.println("LAN_X_CV_WRITE"); 
         #endif
         byte CV_MSB = packetBuffer[6];
         byte CV_LSB = packetBuffer[7];
         byte value = packetBuffer[8]; 
         XpressNet.writeCVMode(CV_LSB+1, value);
       }
       break;             
     case 0x43:
       #if defined(DEBUG) 
         Debug.println("LAN_X_GET_TURNOUT_INFO"); 
       #endif
       XpressNet.getTrntInfo(packetBuffer[5], packetBuffer[6]);
       break;             
     case 0x53:
       #if defined(DEBUG) 
         Debug.println("LAN_X_SET_TURNOUT"); 
       #endif
       XpressNet.setTrntPos(packetBuffer[5], packetBuffer[6], packetBuffer[7]); //change by André Schenk
       break;  
     case 0x80:
       #if defined(DEBUG) 
         Debug.println("LAN_X_SET_STOP"); 
       #endif
       XpressNet.setPower(csEmergencyStop);
       break;  
     case 0xE3:
       if (packetBuffer[5] == 0xF0) {  //DB0
/*          #if defined(DEBUG) 
           Debug.print("LAN_X_GET_LOCO_INFO: "); 
           Debug.println(word(packetBuffer[6] & 0x3F, packetBuffer[7]));  //mit F1-F12
         #endif  */
         //Antwort: LAN_X_LOCO_INFO  Adr_MSB - Adr_LSB
         XpressNet.getLocoInfo(packetBuffer[6] & 0x3F, packetBuffer[7]);
         XpressNet.getLocoFunc(packetBuffer[6] & 0x3F, packetBuffer[7]);  //F13 bis F28
       }
       break;  
     case 0xE4:
       if (packetBuffer[5] == 0xF8) {  //DB0
         //LAN_X_SET_LOCO_FUNCTION  Adr_MSB        Adr_LSB            Type (EIN/AUS/UM)      Funktion
         XpressNet.setLocoFunc(packetBuffer[6] & 0x3F, packetBuffer[7], packetBuffer[8] >> 5, packetBuffer[8] & B00011111); 
       }
       else {
         //LAN_X_SET_LOCO_DRIVE            Adr_MSB          Adr_LSB      DB0          Dir+Speed
         XpressNet.setLocoDrive(packetBuffer[6] & 0x3F, packetBuffer[7], packetBuffer[5] & B11, packetBuffer[8]);       
         #if defined(DEBUG) 
             Debug.print("Z21 Drive: ");
           Debug.print(word(packetBuffer[6] & 0x3F, packetBuffer[7]));
           Debug.print(", ");
           Debug.print(packetBuffer[8]);
           Debug.print("-");
           Debug.println(packetBuffer[5] & B11);
           #endif
       }
       break;  
     case 0xE6:
       if (packetBuffer[5] == 0x30) {  //DB0
         byte Option = packetBuffer[8] & B11111100;  //Option DB3
         byte Adr_MSB = packetBuffer[6] & 0x3F;  //DB1
         byte Adr_LSB = packetBuffer[7];    //DB2
         int CVAdr = packetBuffer[9] | ((packetBuffer[8] & B11) << 7);
         if (Option == 0xEC) {
           #if defined(DEBUG) 
             Debug.println("LAN_X_CV_POM_WRITE_BYTE"); 
           #endif
           byte value = packetBuffer[10];  //DB5
         }
         if (Option == 0xE8) {
           #if defined(DEBUG) 
             Debug.println("LAN_X_CV_POM_WRITE_BIT"); 
           #endif
           //Nicht von der APP Unterstützt
         }
       }
       break;  
     case 0xF1:
       #if defined(DEBUG) 
         Debug.println("LAN_X_GET_FIRMWARE_VERSION"); 
       #endif
       data[0] = 0xf3;
       data[1] = 0x0a;
       data[2] = 0x01;   //V_MSB
       data[3] = 0x23;  //V_LSB
       EthSend (0x09, 0x40, data, true, 0x00);
       break;     
     }
     break; 
     case (0x50):
       #if defined(DEBUG) 
         Debug.print("LAN_SET_BROADCASTFLAGS: "); 
         Debug.println(packetBuffer[4], BIN); // 1=BC Power, Loco INFO, Trnt INFO; 2=BC Änderungen der Rückmelder am R-Bus
       #endif   
       addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3], packetBuffer[4]);
       notifyXNetPower (XpressNet.getPower());  //Zustand Gleisspannung Antworten
     break;
     case (0x51):
       #if defined(DEBUG) 
         Debug.println("LAN_GET_BROADCASTFLAGS"); 
       #endif
       data[0] = 0x00;
       data[1] = 0x00;
       data[2] = 0x00;   
       data[3] = addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3], 0);  
       EthSend (0x08, 0x51, data, false, 0x00); 
     break;
     case (0x60):
       #if defined(DEBUG) 
         Debug.println("LAN_GET_LOCOMODE"); 
       #endif
     break;
     case (0x61):
       #if defined(DEBUG) 
         Debug.println("LAN_SET_LOCOMODE"); 
       #endif
     break;
     case (0x70):
       #if defined(DEBUG) 
         Debug.println("LAN_GET_TURNOUTMODE"); 
       #endif
     break;
     case (0x71):
       #if defined(DEBUG) 
         Debug.println("LAN_SET_TURNOUTMODE"); 
       #endif
     break;
     case (0x81):
       #if defined(DEBUG) 
         Debug.println("LAN_RMBUS_GETDATA"); 
       #endif
    //    S88sendon = 'm';    //Daten werden gemeldet!
    //    notifyS88Data();
     break;
     case (0x82):
       #if defined(DEBUG) 
         Debug.println("LAN_RMBUS_PROGRAMMODULE"); 
       #endif  
     break;
     case (0x85):
       #if defined(DEBUG) 
         Debug.println("LAN_SYSTEMSTATE_GETDATA");  //LAN_SYSTEMSTATE_DATACHANGED
       #endif  
       data[0] = 0x00;  //MainCurrent mA
       data[1] = 0x00;  //MainCurrent mA
       data[2] = 0x00;  //ProgCurrent mA
       data[3] = 0x00;  //ProgCurrent mA        
       data[4] = 0x00;  //FilteredMainCurrent
       data[5] = 0x00;  //FilteredMainCurrent
       data[6] = 0x00;  //Temperature
       data[7] = 0x20;  //Temperature
       data[8] = 0x0F;  //SupplyVoltage
       data[9] = 0x00;  //SupplyVoltage
       data[10] = 0x00;  //VCCVoltage
       data[11] = 0x03;  //VCCVoltage
       data[12] = XpressNet.getPower();  //CentralState
       data[13] = 0x00;  //CentralStateEx
       data[14] = 0x00;  //reserved
       data[15] = 0x00;  //reserved
       EthSend (0x14, 0x84, data, false, 0x00);
     break;
     case (0x89):
       #if defined(DEBUG) 
         Debug.println("LAN_RAILCOM_GETDATA"); 
       #endif  
     break;
     case (0xA0):
       #if defined(DEBUG) 
         Debug.println("LAN_LOCONET_RX"); 
       #endif
     break;
     case (0xA1):
       #if defined(DEBUG) 
         Debug.println("LAN_LOCONET_TX"); 
       #endif
     break;
     case (0xA2):
       #if defined(DEBUG) 
         Debug.println("LAN_LOCONET_FROM_LAN"); 
       #endif
     break;
     case (0xA3):
       #if defined(DEBUG) 
         Debug.println("LAN_LOCONET_DISPATCH_ADDR"); 
       #endif  
     break;
     case (0xA4):
       #if defined(DEBUG) 
         Debug.println("LAN_LOCONET_DETECTOR");   
       #endif  
     break;
   default:
     #if defined(DEBUG) 
       Debug.print("LAN_UNKNOWN_COMMAND 0x"); 
       Debug.println(header, HEX);
     #endif
     data[0] = 0x61;
     data[1] = 0x82;
     EthSend (0x07, 0x40, data, true, 0x00);
   }
 }
}

//--------------------------------------------------------------------------------------------
void EthSend (unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR, byte BC) {
 if (BC != 0x00) {
   IPAddress IPout = Udp.remoteIP();
   for (int i = 0; i < maxIP; i++) {
     if (ActIP[i].time > 0 && ActIP[i].BCFlag >= BC) {    //Noch aktiv?
       IPout[0] = ActIP[i].ip0;
       IPout[1] = ActIP[i].ip1;
       IPout[2] = ActIP[i].ip2;
       IPout[3] = ActIP[i].ip3;
       Udp.beginPacket(IPout, Udp.remotePort());    //Broadcast
       Ethwrite (DataLen, Header, dataString, withXOR);
       Udp.endPacket();
     }
   }
 }
 else {
   Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());    //Broadcast
   Ethwrite (DataLen, Header, dataString, withXOR);
   Udp.endPacket();
 }
}

//--------------------------------------------------------------------------------------------
//Senden von Lokdaten via Ethernet
void Ethwrite (unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR) {
 Udp.write(DataLen & 0xFF);
 Udp.write(DataLen >> 8);
 Udp.write(Header & 0xFF);
 Udp.write(Header >> 8);

 unsigned char XOR = 0;
 byte ldata = DataLen-5;  //Ohne Length und Header und XOR
 if (!withXOR)    //XOR vorhanden?
   ldata++;
 for (int i = 0; i < (ldata); i++) {
   XOR = XOR ^ *dataString;
   Udp.write(*dataString);
   dataString++;
 }
 if (withXOR)
   Udp.write(XOR);
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower (uint8_t State)
{
 byte data[] = { 0x61, 0x00  };
 switch (State) {
 case csNormal: data[1] = 0x01;
   break;
 case csTrackVoltageOff: data[1] = 0x00;
   break;
 case csServiceMode: data[1] = 0x02;
   break;
 case csShortCircuit: data[1] = 0x08;
   break;
 case csEmergencyStop:
   data[0] = 0x81;
   data[1] = 0x00; 
   break;
 default: return;  
 }
 EthSend(0x07, 0x40, data, true, 0x01);
 
 #if defined(DEBUG) 
 Debug.print("XNet Power: ");   
 Debug.println(State, HEX);
 #endif  
}

//--------------------------------------------------------------------------------------------
void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low, uint8_t F2, uint8_t F3 ) {
 #if defined(DEBUG) 
 // Debug.print("Loco Fkt: "); 
 // Debug.print(Adr_Low); 
 // Debug.print(", Fkt2: "); 
 // Debug.print(F2, BIN); 
 // Debug.print("; "); 
 // Debug.println(F3, BIN); 
 #endif
}

//--------------------------------------------------------------------------------------------
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req ) {
 byte DB2 = Steps;
 if (DB2 == 3)  //nicht vorhanden!
   DB2 = 4;
 if (Busy) 
   bitWrite(DB2, 3, 1);
 byte DB3 = Speed;
 if (Direction == 1)  
   bitWrite(DB3, 7, 1);
 byte data[9]; 
 data[0] = 0xEF;  //X-HEADER
 data[1] = Adr_High & 0x3F;
 data[2] = Adr_Low;
 data[3] = DB2;
 data[4] = DB3;
 data[5] = F0;    //F0, F4, F3, F2, F1
 data[6] = F1;    //F5 - F12; Funktion F5 ist bit0 (LSB)
 data[7] = F2;  //F13-F20
 data[8] = F3;  //F21-F28
 if (Req == false)  //kein BC
   EthSend (14, 0x40, data, true, 0x00);  //Send Power und Funktions ask App
 else EthSend (14, 0x40, data, true, 0x01);  //Send Power und Funktions to all active Apps 
}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) {
 #if defined(DEBUG) 
 // Debug.print("Weiche: "); 
 // Debug.print(word(Adr_High, Adr_Low)); 
 // Debug.print(", Position: "); 
 // Debug.println(Pos, BIN); 
 //LAN_X_TURNOUT_INFO
 #endif
 byte data[4];
 data[0] = 0x43;  //HEADER
 data[1] = Adr_High;
 data[2] = Adr_Low;
 data[3] = Pos;
 //EthSend (0x09, 0x40, data, true, 0x01);  
 //change by André Schenk
 EthSend (0x09, 0x40, data, true, 0x00);  
}

//--------------------------------------------------------------------------------------------
void notifyCVInfo(uint8_t State ) {
 #if defined(DEBUG) 
 // Debug.print("CV Prog STATE: "); 
 // Debug.println(State); 
 #endif
 if (State == 0x01 || State == 0x02) {  //Busy or No Data
   //LAN_X_CV_NACK
   byte data[2];
   data[0] = 0x61;  //HEADER
   data[1] = 0x13; //DB0
   EthSend (0x07, 0x40, data, true, 0x00);  
 }
}

//--------------------------------------------------------------------------------------------
void notifyCVResult(uint8_t cvAdr, uint8_t cvData ) {
 #if defined(DEBUG) 
 // Debug.print("CV Prog Read: "); 
 // Debug.print(cvAdr); 
 // Debug.print(", "); 
 // Debug.println(cvData); 
 #endif
 //LAN_X_CV_RESULT
 byte data[5];
 data[0] = 0x64; //HEADER
 data[1] = 0x14;  //DB0
 data[2] = 0x00;  //CVAdr_MSB
 data[3] = cvAdr;  //CVAdr_LSB
 data[4] = cvData;  //Value
 EthSend (0x0A, 0x40, data, true, 0x00);
}

// // -------------------------------------------------------------- 
// void SetupS88() {
//  S88Module = EEPROM.read(EES88Moduls);
//  if (S88Module > 62 || S88Module == 0) { //S88 off!
//    S88Module = 0;
//    TCCR2B = 0<<CS22 | 0<<CS21 | 0<<CS20;  //Timer 2 off
//    return;
//  }
//  //S88 Aktivieren!

//  //Setup Timer2.
//  //Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
//  //Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
//  /* 
//   16Mhz / 1 prescaler = 16Mhz = CS 001 
//   16Mhz / 8 prescaler = 2MHz oder 0,5usec = CS 010
//   16Mhz / 64 prescaler = 250kHz = CS 011 
//   16Mhz / 256 prescaler = CS 100
//   16Mhz / 1024 prescaler = CS 101
//   */
//  //Timer2 Settings: Timer Prescaler /256
//  //Timmer clock = 16MHz/256
//  TCCR2A = 0;
//  TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20;
//  TIMSK2 = 1<<TOIE2; //Timer2 Overflow Interrupt Enable
//  TCNT2=TIMER_Time; //load the timer for its first cycle
// }

// //-------------------------------------------------------------- 
// //Timer ISR Routine
// //Timer2 overflow Interrupt vector handler
// ISR(TIMER2_OVF_vect) {
//  if (S88RCount == 3)    //Load/PS Leitung auf 1, darauf folgt ein Schiebetakt nach 10 ticks!
//    digitalWrite(S88PSPin, HIGH);
//  if (S88RCount == 4)   //Schiebetakt nach 5 ticks und S88Module > 0
//    digitalWrite(S88ClkPin, HIGH);       //1. Impuls
//  if (S88RCount == 5)   //Read Data IN 1. Bit und S88Module > 0
//    S88readData();    //LOW-Flanke während Load/PS Schiebetakt, dann liegen die Daten an
//  if (S88RCount == 9)    //Reset-Plus, löscht die den Paralleleingängen vorgeschaltetetn Latches
//    digitalWrite(S88ResetPin, HIGH);
//  if (S88RCount == 10)    //Ende Resetimpuls
//    digitalWrite(S88ResetPin, LOW);
//  if (S88RCount == 11)    //Ende PS Phase
//    digitalWrite(S88PSPin, LOW);
//  if (S88RCount >= 12 && S88RCount < 10 + (S88Module * 8) * 2) {    //Auslesen mit weiteren Schiebetakt der Latches links
//    if (S88RCount % 2 == 0)      //wechselnder Taktimpuls/Schiebetakt
//      digitalWrite(S88ClkPin, HIGH);  
//    else S88readData();    //Read Data IN 2. bis (Module*8) Bit
//  }
//  S88RCount++;      //Zähler für Durchläufe/Takt
//  if (S88RCount >= 10 + (S88Module * 8) * 2) {  //Alle Module ausgelesen?
//    S88RCount = 0;                    //setzte Zähler zurück
//    S88RMCount = 0;                  //beginne beim ersten Modul von neuem
//    //init der Grundpegel
//    digitalWrite(S88PSPin, LOW);    
//    digitalWrite(S88ClkPin, LOW);
//    digitalWrite(S88ResetPin, LOW);
//    if (S88sendon == 's')  //Änderung erkannt
//      S88sendon = 'i';      //senden
//  }
//  //Capture the current timer value. This is how much error we have due to interrupt latency and the work in this function
//  TCNT2 = TCNT2 + TIMER_Time;    //Reload the timer and correct for latency.
// }

// //--------------------------------------------------------------
// //Einlesen des Daten-Bit und Vergleich mit vorherigem Durchlauf
// void S88readData() {
//  digitalWrite(S88ClkPin, LOW);  //LOW-Flanke, dann liegen die Daten an
//  byte Modul = S88RMCount / 8;
//  byte Port = S88RMCount % 8;
//  byte getData = digitalRead(S88DataPin);  //Bit einlesen
//  if (bitRead(data[Modul],Port) != getData) {     //Zustandsänderung Prüfen?
//    bitWrite(data[Modul],Port,getData);          //Bitzustand Speichern
//    S88sendon = 's';  //Änderung vorgenommen. (SET)
//  }
//  S88RMCount++;
// }

// //--------------------------------------------------------------------------------------------
// void notifyS88Data() {
//  if (S88sendon == 'i' || S88sendon == 'm') {
//    byte MAdr = 1;  //Rückmeldemodul
//    byte datasend[11];  //Array Gruppenindex (1 Byte) & Rückmelder-Status (10 Byte)
//    datasend[0] = 0; //Gruppenindex für Adressen 1 bis 10
//    for(byte m = 0; m < S88Module; m++) {  //Durchlaufe alle aktiven Module im Speicher
//      datasend[MAdr] = data[m];
//      MAdr++;  //Nächste Modul in der Gruppe
//      if (MAdr >= 11) {  //10 Module à 8 Ports eingelesen
//        MAdr = 1;  //beginne von vorn
//        EthSend (0x0F, 0x80, datasend, false, 0x02); //RMBUS_DATACHANED
//        datasend[0]++; //Gruppenindex erhöhen
//      }
//    }
//    if (MAdr < 11) {  //noch unbenutzte Module in der Gruppe vorhanden? Diese 0x00 setzten und dann Melden!
//      while (MAdr < 11) {
//        datasend[MAdr] = 0x00;  //letzten leeren Befüllen
//        MAdr++;   //Nächste Modul in der Gruppe   
//      }
//      EthSend (0x0F, 0x80, datasend, false, 0x02); //RMBUS_DATACHANED
//    }
//    S88sendon = '0';        //Speicher Rücksetzten
//  }
// }

//--------------------------------------------------------------------------------------------
#if defined(DEBUG)
int freeRam () {
//  extern int __heap_start, *__brkval;
//  int v;
//  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  return ESP.getFreeHeap();
}
#endif


