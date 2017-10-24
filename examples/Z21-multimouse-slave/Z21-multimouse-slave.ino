/*
   Z21 emulator for Roco command station 10764 and Roco multiMOUSE.

   Version: 04
   Works with XpressNet library version 1.7 (29.09.2015) from Philipp Gahtow (https://sourceforge.net/projects/pgahtow/files/Arduino%20%28v1.0%29%20libaries/XpressNet.zip )

  "Fathers":
  Initital development: http://pgahtow.de/wiki/index.php?title=XpressNet by Philipp Gahtow.
  Further development which this sketch based on: https://github.com/schabauerj/Roco_Z21_Arduino by Markus Heller

  Russian support tread: http://forum.modelldepo.ru/showthread.php?t=17157

  Update history by BR95009:
  29.09.2015 - V04 - Buttons F13-F28 fixed by Philipp Ghatow in the library.
  27.09.2015 - V03 - retrieved feedback Multimouse => Arduino => Client.
  27.09.2015 - V02 - Steps management fixed temporary.
  20.09.2015 - V02 - added array to keep buttons state for different loks
  17.09.2015  - Feedback emu z21=>Clients Apps added.  Now Speed gauge and buttons in Apps work correct.  Also direction switch works correct. Sending to all devices
                But buttons work only for current lok-need to fix.

  Hardware:
  1) Arduino Mega 2560 - http://www.banggood.com/Mega2560-R3-ATmega2560-16AU-Control-Board-With-USB-Cable-For-Arduino-p-73020.html
  2) Ethernet Shield W5100  - http://www.banggood.com/Ethernet-Shield-Module-W5100-Micro-SD-Card-Slot-For-Arduino-UNO-MEGA-p-908461.html
  3) Protoshield to solder RS485 interface bord on - http://www.banggood.com/Arduino-Compatible-328-ProtoShield-Prototype-Expansion-Board-p-926451.html
  4) RS485 interface board WaveShare (http://www.chipdip.ru/product/rs485-board-5v/ ) to realize RS485 interface to connect to S-Bus slave of Roco command station:
    RS486 board pinout:
    VCC => 5V of arduino;
    GND => GND of arduino
    RO => TX1 (18) pin (or viseversa, sheck if problem).
    DI => RX1 (19) pin (or viseversa, sheck if problem).
    RSE => Pin 3 (Digital)

  WARNING: jumpers on the board RS485 to be soldered as A3 and B2!

  MAX485 RX/TX are connected to Serial1 pins of MEGA (18/19) => you can use Serial0 for debug log in Serial Terminal window of Arduino IDE app.
  Control PIN (RSE) connected to pin 3.

  LED on pin 13 is indicator of XpressNet connection: flashes - no connection, solid - OK: in notifyXNetStatus(uint8_t LedState)

  GENERAL Connections and adjustments (how to run):
  1) Plug XpressNet cable from z21 emulator to Slave socket of your ROCO command station (10764). Your multiMouse you should plug as Master.
  2) LAN cable plug to W5100 socket and to WLAN router (use separated with factory settings)
  3) Make WLAN on router with IP address space 192.168.0.*
  4) Download Z21 app for Android/iOS
  5) Connect to router WIFI. Check IP adress space for 192.168.0.* in Ipad.
  6) Edit your MAC adsress of ethernet shield below in sketch (you have to know it)
  7) Edit IP adress below in sketch (I use 192.168.0.2 for instance)
  8) Load the sketch to Mega.
  9) Run Serial terminal window and check IP adress at start of z21 emulator.
  10) Put this address to settings of your z21 app.
  11) All should work.

  Current known ISSUES(to do):
  - Client app send strange adreses to get info about lok. Find out why.

  */

// connect XpressNet lib
#include <esp8266-XpressNet.h>
XpressNetClass XpressNet;

#include <SPI.h> // needed for Arduino versions later than 0018
#include <ESP8266WiFi.h>

// WiFiManager auto-configuration library
#include <DNSServer.h>        //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h> //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>      //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <WiFiUdp.h> // UDP library
#include <EEPROM.h>

#define EEip 10  //Startddress im EEPROM für die IP
#define EEXNet 9 //Adresse im XNet-Bus

// Defines form z21.h file:
extern void z21Receive();
extern void z21CheckActiveIP();
extern void z21Setup();
extern XpressNetClass XpressNet;

#define LAN_GET_SERIAL_NUMBER 0x10
#define LAN_GET_CONFIG 0x12
#define LAN_GET_HWINFO 0x1A
#define LAN_LOGOFF 0x30
#define LAN_XPRESS_NET 0x40
#define LAN_X_GENERAL 0x21
#define LAN_X_GET_VERSION 0x21
#define LAN_X_GET_STATUS 0x24
#define LAN_X_SET_TRACK_POWER_OFF 0x80
#define LAN_X_SET_TRACK_POWER_ON 0x81
#define LAN_X_CV_READ_0 0x23
#define LAN_X_CV_READ_1 0x11
#define LAN_X_CV_WRITE_0 0x24
#define LAN_X_CV_WRITE_1 0x12
#define LAN_X_GET_TURNOUT_INFO 0x43
#define LAN_X_SET_TURNOUT 0x53
#define LAN_X_SET_STOP 0x80
#define LAN_X_GET_LOCO_INFO_0 0xE3
#define LAN_X_GET_LOCO_INFO_1 0xF0
#define LAN_X_SET_LOCO_FUNCTION_0 0xE4
#define LAN_X_SET_LOCO_FUNCTION_1 0xF8
#define LAN_X_CV_POM 0xE6
#define LAN_X_CV_POM_WRITE 0x30
#define LAN_X_CV_POM_WRITE_BYTE 0xEC
#define LAN_X_CV_POM_WRITE_BIT 0xE8
#define LAN_X_GET_FIRMWARE_VERSION 0xF1
#define LAN_SET_BROADCASTFLAGS 0x50
#define LAN_GET_BROADCASTFLAGS 0x51
#define LAN_GET_LOCOMODE 0x60
#define LAN_SET_LOCOMODE 0x61
#define LAN_GET_TURNOUTMODE 0x70
#define LAN_SET_TURNOUTMODE 0x71
#define LAN_RMBUS_GETDATA 0x81
#define LAN_RMBUS_PROGRAMMODULE 0x82
#define LAN_SYSTEMSTATE_GETDATA 0x85
#define LAN_RAILCOM_GETDATA 0x89
#define LAN_LOCONET_FROM_LAN 0xA2
#define LAN_LOCONET_DISPATCH_ADDR 0xA3

// End defines of z21. file ==================

// CPP defines:
byte XBusVer = 0x30;

// buffers for receiving and sending data
unsigned char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

// IP structure definitions:
#define maxIP 10      //Speichergröße für IP-Adressen
#define ActTimeIP 20  //Aktivhaltung einer IP für (sec./2)
#define interval 2000 //Check active IP every 2 seconds.
struct TypeActIP
{
  byte ip0;  // Byte IP
  byte ip1;  // Byte IP
  byte ip2;  // Byte IP
  byte ip3;  // Byte IP
  byte time; //Time
};

TypeActIP ActIP[maxIP]; //Speicherarray für IPs

// An EthernetUDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
unsigned int localPort = 21105; // local port to listen on
// end of CPP defines ==========

// Define MAC address of your Ethernet shield:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // Check with your W5100 shield MAC address!!!

// IP address of z21. Depends on your LAN configuration. Should be the same sub net.
IPAddress ip(192, 168, 0, 2);

//Run ethernet shield in server mode
WiFiServer server(80);

#define ResetPin 0 //Reset Pin press to set to default IP when restarting!

// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.

byte XNetAddress = 30; //Adresse im XpressNet

#define interval 2000 //interval at milliseconds

long previousMillis = 0; // will store last time of IP decount updated

// Pin 13 has an LED connected on most Arduino boards, for notifyXNetStatus(uint8_t LedState)
// give it a name:
int led = 13;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// S E T U P
void setup()
{

  Serial.begin(115200); // Debug via terminal window at 9600
  Serial.println("Z21 - v04");
  pinMode(ResetPin, INPUT);     //define reset button pin as input pin
  digitalWrite(ResetPin, HIGH); //PullUp to HIGH
  delay(100);

  // // EEPROM usage for storing the IP adresses
  // if (digitalRead(ResetPin) == LOW || EEPROM.read(EEXNet) < 32) {
  //   XNetAddress = EEPROM.read(EEXNet);
  // }
  // else {
  //   EEPROM.write(EEXNet, XNetAddress);
  //   EEPROM.write(EEip, ip[0]);
  //   EEPROM.write(EEip + 1, ip[1]);
  //   EEPROM.write(EEip + 2, ip[2]);
  //   EEPROM.write(EEip + 3, ip[3]);
  // }
  // ip[0] = EEPROM.read(EEip);
  // ip[1] = EEPROM.read(EEip + 1);
  // ip[2] = EEPROM.read(EEip + 2);
  // ip[3] = EEPROM.read(EEip + 3);

  // // start the Ethernet and UDP:
  // Ethernet.begin(mac, ip);

  // Start WiFi
  WiFiManager wifiManager;
  wifiManager.autoConnect("Z21-Config");
  ip = WiFi.localIP();

  Serial.print("Starting Ethernet at IP address:");
  Serial.println(ip);
  Serial.println("Starting XPressNet");
  XpressNet.start(XNetAddress, 5, true); // Start XpressNet lib with XnetAddress and PIN 3 as control pin. If you use Mega, then XpressNet lin use Serial1 for MAX485 communication
  Serial.println("Starting Z21 Emulation");
  z21Setup();
  Serial.println("Setup finished");
}

// end of SETUP

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// L O O P
void loop()
{
  z21Receive();
  XpressNet.receive(); // XpressNet work function: read and write to X-net bus.
  z21Receive();
  XpressNet.receive(); // XpressNet work function: read and write to X-net bus.

  //Check active IP every 2 seconds.
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    z21CheckActiveIP();
  }
}

// end of L O O P ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Functions part:

/**********************************************************
 * HELPERS
 * ********************************************************/
//--------------------------------------------------------------------------------------------
void clearIPSlots()
{
  for (int i = 0; i < maxIP; i++)
  {
    ActIP[i].ip0 = 0;
    ActIP[i].ip1 = 0;
    ActIP[i].ip2 = 0;
    ActIP[i].ip3 = 0;
    ActIP[i].time = 0;
  }
}

String printIP(byte *ip)
{
  static String ips;
  ips = String(ip[0]);
  ips += '.';
  ips += ip[1];
  ips += '.';
  ips += ip[2];
  ips += '.';
  ips += ip[3];
  return ips;
}

//--------------------------------------------------------------------------------------------
void addIPToSlot(byte ip0, byte ip1, byte ip2, byte ip3)
{
  byte Slot = maxIP;
  for (int i = 0; i < maxIP; i++)
  {
    if (ActIP[i].ip0 == ip0 && ActIP[i].ip1 == ip1 && ActIP[i].ip2 == ip2 && ActIP[i].ip3 == ip3)
    {
      ActIP[i].time = ActTimeIP;
      //Serial.println("IP '" + printIP(&ActIP[i].ip0) + "' was already active");
      return;
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
}

//--------------------------------------------------------------------------------------------
//Send out data via Ethernet
void EthSendOut(unsigned int DataLen, unsigned int Header, byte Data[], boolean withXOR)
{
  Udp.write(DataLen & 0xFF);
  Udp.write(DataLen & 0xFF00);
  Udp.write(Header & 0xFF);
  Udp.write(Header & 0xFF00);

  unsigned char XOR = 0;
  byte ldata = DataLen - 5; //Withiut Length und Header und XOR
  if (!withXOR)             //XOR present?
    ldata++;
  for (int i = 0; i < (ldata); i++)
  {
    XOR = XOR ^ Data[i];
    Udp.write(Data[i]);
  }
  if (withXOR)
    Udp.write(XOR);
}

//--------------------------------------------------------------------------------------------
void EthSend(unsigned int DataLen, unsigned int Header, byte Data[], boolean withXOR, boolean BC)
{
  if (BC)
  {
    IPAddress IPout = Udp.remoteIP();
    for (int i = 0; i < maxIP; i++)
    {
      if (ActIP[i].time > 0)
      { //Still aktiv?
        IPout[0] = ActIP[i].ip0;
        IPout[1] = ActIP[i].ip1;
        IPout[2] = ActIP[i].ip2;
        IPout[3] = ActIP[i].ip3;
        Udp.beginPacket(IPout, Udp.remotePort()); //Broadcast
        EthSendOut(DataLen, Header, Data, withXOR);
        Udp.endPacket();
      }
    }
  }
  else
  {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //Broadcast
    EthSendOut(DataLen, Header, Data, withXOR);
    Udp.endPacket();
  }
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower(uint8_t State)
{
  Serial.print("XNet Power: 0x");
  Serial.print(State, HEX);

  byte data[] = {0x61, 0x00};
  switch (State)
  {
  case csNormal:
    data[1] = 0x01;
    Serial.println(" => ON");
    break;
  case csTrackVoltageOff:
    data[1] = 0x00;
    Serial.println(" => Voltage OFF");
    break;
  case csServiceMode:
    data[1] = 0x02;
    Serial.println(" => Service Mode");
    break;
  case csEmergencyStop:
    data[0] = 0x81;
    data[1] = 0x00;
    Serial.println(" => Emergency Stop");
    break;
  }
  EthSend(0x07, 0x40, data, true, true);
}

//--------------------------------------------------------------------------------------------
void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low, uint8_t F2, uint8_t F3)
{
  Serial.println("Loco Fkt: ");
  Serial.println(Adr_Low);
  Serial.println(", Fkt2: ");
  Serial.println(F2, BIN);
  Serial.println("; ");
  Serial.println(F3, BIN);
}

//--------------------------------------------------------------------------------------------
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req)
{
  byte DB2 = Steps;
  if (DB2 == 3) //unavailable!
    DB2 = 4;
  if (Busy)
    bitWrite(DB2, 3, 1);
  byte DB3 = Speed;
  if (Direction == 1)
    bitWrite(DB3, 7, 1);
  byte data[9];
  data[0] = 0xEF; //X-HEADER
  data[1] = Adr_High & 0x3F;
  data[2] = Adr_Low;
  data[3] = DB2;                       // steps
  data[4] = DB3;                       //speed
  data[5] = F0;                        //F0, F4, F3, F2, F1
  data[6] = F1;                        //F5 - F12; Funktion F5 is bit0 (LSB)
  data[7] = F2;                        //F13-F20
  data[8] = F3;                        //F21-F28
  EthSend(14, 0x40, data, true, true); //Send Power und Funktions to all active Apps

  /*
  Serial.print("notifyLokAll(): ADDR_HI: ");
  Serial.print(data[1], DEC);
  Serial.print(", ADDR_LO: ");
  Serial.print(data[2], DEC);
  Serial.print(", STEPS: ");
  Serial.print(data[3], DEC);
  Serial.print(", Speed: ");
  Serial.print(data[4], BIN);
  Serial.print(", F1-4: ");
  Serial.print(data[5], BIN);
  Serial.print(", F5-12: ");
  Serial.print(data[6], BIN);
  Serial.print(", F13-20: ");
  Serial.print(data[7], BIN);
  Serial.print(", F11-28: ");
  Serial.println(data[8], BIN);
  */
}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos)
{
  Serial.print("TurnOut: ");
  Serial.print(word(Adr_High, Adr_Low));
  Serial.print(", Position: ");
  Serial.println(Pos, BIN);

  //LAN_X_TURNOUT_INFO
  byte data[4];
  data[0] = 0x43; //HEADER
  data[1] = Adr_High;
  data[2] = Adr_Low;
  data[3] = Pos;
  EthSend(0x09, 0x40, data, true, false);
}

//--------------------------------------------------------------------------------------------
void notifyCVInfo(uint8_t State)
{
  // Serial.print("CV Prog STATE: ");
  // Serial.println(State);
  if (State == 0x01 || State == 0x02)
  { //Busy or No Data

    //LAN_X_CV_NACK
    byte data[2];
    data[0] = 0x61; //HEADER
    data[1] = 0x13; //DB0
    EthSend(0x07, 0x40, data, true, false);
  }
}

//--------------------------------------------------------------------------------------------
void notifyCVResult(uint8_t cvAdr, uint8_t cvData)
{
  //LAN_X_CV_RESULT
  byte data[5];
  data[0] = 0x64;   //HEADER
  data[1] = 0x14;   //DB0
  data[2] = 0x00;   //CVAdr_MSB
  data[3] = cvAdr;  //CVAdr_LSB
  data[4] = cvData; //Value
  EthSend(0x0A, 0x40, data, true, false);
}

//--------------------------------------------------------------------------------------------
void notifyXNetVersion(uint8_t Version, uint8_t ID)
{
  XBusVer = Version;
}

//--------------------------------------------------------------------------------------------
void notifyXNetStatus(uint8_t LedState)
{
  digitalWrite(led, LedState);
}

// Parser functon to read X-Bus input and send to z21 Client App
void xPressNetParse(byte *packetBuffer, byte *data)
{
  boolean ok = false;
  switch (packetBuffer[4])
  { //X-Header
  case LAN_X_GENERAL:
    switch (packetBuffer[5])
    { //DB0
    case LAN_X_GET_VERSION:
      data[0] = 0x63;
      data[1] = 0x21;
      data[3] = XBusVer; //X-Bus Version
      data[4] = 0x12;    //ID der Zentrale
      data[5] = 0;
      EthSend(0x09, 0x40, data, true, false);
      Serial.println("LAN_X_GET_VERSION");
      break;
    case LAN_X_GET_STATUS:
      data[0] = 0x62;
      data[1] = 0x22;
      data[2] = XpressNet.getPower();
      EthSend(0x08, 0x40, data, true, false);
      //Serial.println("LAN_X_GET_STATUS"); This is asked very often ...
      break;
    case LAN_X_SET_TRACK_POWER_OFF:
      ok = XpressNet.setPower(csTrackVoltageOff);
      //Serial.println("LAN_X_SET_TRACK_POWER_OFF");
      if (ok == false)
      {
        //Serial.println("Power Send FEHLER");
      }
      break;
    case LAN_X_SET_TRACK_POWER_ON:
      ok = XpressNet.setPower(csNormal);
      //Serial.println("LAN_X_SET_TRACK_POWER_ON");
      if (ok == false)
      {
        //Serial.println("Power Send FEHLER");
      }
      break;
    }
    break;
  case LAN_X_CV_READ_0:
    if (packetBuffer[5] == LAN_X_CV_READ_1)
    { //DB0
      //Serial.println("LAN_X_CV_READ");
      byte CV_MSB = packetBuffer[6];
      byte CV_LSB = packetBuffer[7];
      XpressNet.readCVMode(CV_LSB + 1);
    }
    break;
  case LAN_X_CV_WRITE_0:
    if (packetBuffer[5] == LAN_X_CV_WRITE_1)
    { //DB0
      //Serial.println("LAN_X_CV_WRITE");
      byte CV_MSB = packetBuffer[6];
      byte CV_LSB = packetBuffer[7];
      byte value = packetBuffer[8];
      XpressNet.writeCVMode(CV_LSB + 1, value);
    }
    break;
  case LAN_X_GET_TURNOUT_INFO:
    Serial.println("LAN_X_GET_TURNOUT_INFO");
    XpressNet.getTrntInfo(packetBuffer[5], packetBuffer[6]);
    break;
  case LAN_X_SET_TURNOUT:
    //Serial.println("LAN_X_SET_TURNOUT");
    XpressNet.setTrntPos(packetBuffer[5], packetBuffer[6], packetBuffer[7] & 0x0F);
    break;
  case LAN_X_SET_STOP:
    //Serial.println("LAN_X_SET_STOP");
    ok = XpressNet.setPower(csEmergencyStop);
    if (ok == false)
    {
      //Serial.println("Power Send FEHLER");
    }
    break;

  case LAN_X_GET_LOCO_INFO_0:

    if (packetBuffer[5] == LAN_X_GET_LOCO_INFO_1)
    { //DB0=

      //Ask Xbus for current loko infor:
      //                  LAN_X_LOCO_INFO  Adr_MSB - Adr_LSB
      XpressNet.getLocoInfo(packetBuffer[6] & 0x3F, packetBuffer[7]); // >> run notifyLokAll()

      //XpressNet.getLocoFunc (packetBuffer[6] & 0x3F, packetBuffer[7]); // >> run notifyLokFunc() run F13 bis F28 //under revision

      Serial.print("LAN_X_GET_LOCO_INFO on address: ");
      Serial.print(packetBuffer[6] & 0x3F, DEC);
      Serial.print(",  ");
      Serial.println(packetBuffer[7], DEC);
    }
    break;

  case LAN_X_SET_LOCO_FUNCTION_0: //SET loco func and speed here:
    if (packetBuffer[5] == LAN_X_SET_LOCO_FUNCTION_1)
    { //DB0 - X Header
      //LAN_X_SET_LOCO_FUNCTION  Adr_MSB-DB1        Adr_LSB-DB2     Type (EIN/AUS/UM) DB3   FunktionDB3
      XpressNet.setLocoFunc(packetBuffer[6] & 0x3F, packetBuffer[7], packetBuffer[8] >> 5, packetBuffer[8] & B00011111);
    }
    else
    {
      //LAN_X_SET_LOCO_DRIVE            Adr_MSB          Adr_LSB      DB0                    Dir+Speed
      XpressNet.setLocoDrive(packetBuffer[6] & 0x3F, packetBuffer[7], packetBuffer[5] & B11, packetBuffer[8]);
    }
    // LAN_X_GET_LOCO_INFO_0 -> Client:
    XpressNet.getLocoInfo(packetBuffer[6] & 0x3F, packetBuffer[7]); // >> run notifyLokAll()
    break;

  case LAN_X_CV_POM:
    Serial.println("LAN_X_CV_POM");
    if (packetBuffer[5] == LAN_X_CV_POM_WRITE)
    {                                            //DB0
      byte Option = packetBuffer[8] & B11111100; //Option DB3
      byte Adr_MSB = packetBuffer[6] & 0x3F;     //DB1
      byte Adr_LSB = packetBuffer[7];            //DB2
      int CVAdr = packetBuffer[9] | ((packetBuffer[8] & B11) << 7);
      if (Option == LAN_X_CV_POM_WRITE_BYTE)
      {
        Serial.println("Client ask: LAN_X_CV_POM_WRITE_BYTE");
        byte value = packetBuffer[10]; //DB5
      }
      if (Option == LAN_X_CV_POM_WRITE_BIT)
      {
        Serial.println("Client ask: LAN_X_CV_POM_WRITE_BIT");
        //Nicht von der APP Unterstützt
      }
    }
    break;
  case LAN_X_GET_FIRMWARE_VERSION:
    Serial.println("Client ask: LAN_X_GET_FIRMWARE_VERSION");
    data[0] = 0xf3;
    data[1] = 0x0a;
    data[3] = 0x01; //V_MSB
    data[4] = 0x23; //V_LSB

    EthSend(0x09, 0x40, data, true, false);
    break;
  }
}

// ENSD of xPressNetParse ///////////////////////////////////

/********************************
 * Public Members
 * *****************************/

void z21Setup()
{
  Udp.begin(localPort); //UDP Z21 Port
  clearIPSlots();       //Delete stored active IP's
}

void z21CheckActiveIP()
{
  for (int i = 0; i < maxIP; i++)
  {
    if (ActIP[i].time > 0)
    {
      ActIP[i].time--; //Zeit herrunterrechnen
    }
    else
    {
      //clear IP DATA
      ActIP[i].ip0 = 0;
      ActIP[i].ip1 = 0;
      ActIP[i].ip2 = 0;
      ActIP[i].ip3 = 0;
      ActIP[i].time = 0;
    }
  }
}

//--------------------------------------------------------------------------------------------
void z21Receive()
{
  int packetSize = Udp.parsePacket();
  if (packetSize > 0)
  {
    addIPToSlot(Udp.remoteIP()[0], Udp.remoteIP()[1], Udp.remoteIP()[2], Udp.remoteIP()[3]);
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); // read the packet into packetBufffer
    // send a reply, to the IP address and port that sent us the packet we received
    int header = (packetBuffer[3] << 8) + packetBuffer[2];
    //    int datalen = (packetBuffer[1]<<8) + packetBuffer[0];
    byte data[16];
    boolean ok = false;
    //packetBuffer[packetSize]= 0;
    //Serial.println("z21 packetBuffer: ", (char*) packetBuffer);
    switch (header)
    {
    case LAN_GET_SERIAL_NUMBER:
      data[0] = 0xF5; //Seriennummer 32 Bit (little endian)
      data[1] = 0x0A;
      data[2] = 0x00;
      data[3] = 0x00;
      EthSend(0x08, 0x10, data, false, false);
      //Serial.println("z21 Serial Number: ", (char*) data);
      Serial.println("Client ask: LAN_GET_SERIAL_NUMBER");
      break;
    case LAN_GET_CONFIG:
      Serial.println("Client ask: Z21-Settings");
      break;
    case LAN_GET_HWINFO:
      data[0] = 0x00; //HwType 32 Bit
      data[1] = 0x00;
      data[2] = 0x02;
      data[3] = 0x01;
      data[4] = 0x20; //FW Version 32 Bit
      data[5] = 0x01;
      data[6] = 0x00;
      data[7] = 0x00;
      EthSend(0x0C, 0x1A, data, false, false);
      Serial.println("Client ask: LAN_GET_HWINFO");
      break;
    case LAN_LOGOFF:
      Serial.println("Client ask: LAN_LOGOFF");
      //Antwort von Z21: keine
      break;
    case LAN_XPRESS_NET:
      xPressNetParse(packetBuffer, data); // PARSE AND send to X-BUS =====================>>
      break;
    case LAN_SET_BROADCASTFLAGS:
      Serial.println("Client ask: LAN_SET_BROADCASTFLAGS: ");
      // //debug.print(packetBuffer[4], BIN);  // 1=BC Power, Loco INFO, Trnt INFO; B100=BC Sytemstate Datachanged
      break;
    case LAN_GET_BROADCASTFLAGS:
      Serial.println("Client ask: LAN_GET_BROADCASTFLAGS");
      break;
    case LAN_GET_LOCOMODE:
      Serial.print("Client ask: LAN_GET_LOCOMODE with address:");
      Serial.println(packetBuffer[7]);
      break;
    case LAN_SET_LOCOMODE:
      Serial.print("Client ask: LAN_SET_LOCOMODE with address: ");
      Serial.println(packetBuffer[7]);
      break;
    case LAN_GET_TURNOUTMODE:
      Serial.println("Client ask: LAN_GET_TURNOUTMODE");
      break;
    case LAN_SET_TURNOUTMODE:
      Serial.println("Client ask: LAN_SET_TURNOUTMODE");
      break;
    case LAN_RMBUS_GETDATA:
      Serial.println("Client ask: LAN_RMBUS_GETDATA");
      break;
    case LAN_RMBUS_PROGRAMMODULE:
      Serial.println("Client ask: LAN_RMBUS_PROGRAMMODULE");
      break;
    case LAN_SYSTEMSTATE_GETDATA:
      Serial.println("Client ask: LAN_SYSTEMSTATE_GETDATA"); //LAN_SYSTEMSTATE_DATACHANGED
      data[0] = 0x00;                                        //MainCurrent mA
      data[1] = 0x00;                                        //MainCurrent mA
      data[2] = 0x00;                                        //ProgCurrent mA
      data[3] = 0x00;                                        //ProgCurrent mA
      data[4] = 0x00;                                        //FilteredMainCurrent
      data[5] = 0x00;                                        //FilteredMainCurrent
      data[6] = 0x00;                                        //Temperature
      data[7] = 0x20;                                        //Temperature
      data[8] = 0x0F;                                        //SupplyVoltage
      data[9] = 0x00;                                        //SupplyVoltage
      data[10] = 0x00;                                       //VCCVoltage
      data[11] = 0x03;                                       //VCCVoltage
      data[12] = XpressNet.getPower();                       //CentralState
      data[13] = 0x00;                                       //CentralStateEx
      data[14] = 0x00;                                       //reserved
      data[15] = 0x00;                                       //reserved
      EthSend(0x14, 0x84, data, false, false);
      break;
    case LAN_RAILCOM_GETDATA:
      Serial.println("Client ask: LAN_RAILCOM_GETDATA");
      break;
    case LAN_LOCONET_FROM_LAN:
      Serial.println("Client ask: LAN_LOCONET_FROM_LAN");
      break;
    case LAN_LOCONET_DISPATCH_ADDR:
      Serial.println("Client ask: LAN_LOCONET_DISPATCH_ADDR");
      break;
    default:
      Serial.println("Client ask: LAN_X_UNKNOWN_COMMAND");
      data[0] = 0x61;
      data[1] = 0x82;
      EthSend(0x07, 0x40, data, true, false);
    }
  }
}
// end of file
