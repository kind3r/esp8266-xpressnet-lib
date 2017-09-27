/*
  Simple XpressNet sniffer for ESP8266


*/

// -----------------------------------------------------------
// XpressNet settings
// -----------------------------------------------------------
// My XpressNet address
#define XNetAddress 30
// RS485 TX pin 
#define XNetRS485_TX 4 
// RS485 RX pin
#define XNetRS485_RX 2
//RS485 send/receive pin
#define XNetRS485_TXRX 5

#include <XpressNet.h>
XpressNetClass XpressNet;

void setup()
{
  // debug output
  Serial.begin(115200);

  XpressNet.start(XNetAddress, XNetRS485_TXRX, true);
  XpressNet.setPower(csNormal);

}

void loop()
{
  XpressNet.receive();
}

void notifyXNetStatus (uint8_t State)
{
  // digitalWrite(led, State);
}

//--------------------------------------------------------------------------------------------
void notifyXNetPower (uint8_t State)
{
  Serial.print("Power: ");
  switch (State) {
    case csNormal: Serial.println("ON"); break;
    case csTrackVoltageOff: Serial.println("OFF"); break;
    case csEmergencyStop: Serial.println("EmStop"); break;
    case csShortCircuit: Serial.println("SHORT"); break;
    case csServiceMode: Serial.println("PROG"); break;
  }
}

//--------------------------------------------------------------------------------------------
void notifyXNetVersion(uint8_t Version, uint8_t ID ) 
{
  Serial.print("Version: ");
  Serial.println(Version, HEX);
}

//--------------------------------------------------------------------------------------------
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req ) {
   Serial.print(Busy); 
   Serial.print(" Loco ALL: "); 
   Serial.print(Adr_Low); 
   Serial.print(", Steps: "); 
   Serial.print(Steps, BIN); 
   Serial.print(", Speed: "); 
   Serial.print(Speed); 
   Serial.print(", Direction: "); 
   Serial.print(Direction); 
   Serial.print(", Fkt: "); 
   Serial.print(F0, BIN); 
   Serial.print("; "); 
   Serial.print(F1, BIN); 
   Serial.print(", Fkt2: "); 
   Serial.print(F2, BIN); 
   Serial.print("; "); 
   Serial.println(F3, BIN); 
}

//--------------------------------------------------------------------------------------------
void notifyLokFunc(uint8_t Adr_High, uint8_t Adr_Low, uint8_t F2, uint8_t F3 ) {
  Serial.print("Loco Fkt: "); 
  Serial.print(Adr_Low); 
  Serial.print(", Fkt2: "); 
  Serial.print(F2, BIN); 
  Serial.print("; "); 
  Serial.println(F3, BIN); 
}

//--------------------------------------------------------------------------------------------
void notifyTrnt(uint8_t Adr_High, uint8_t Adr_Low, uint8_t Pos) {
  Serial.print("Turnout: "); 
  Serial.print(word(Adr_High, Adr_Low)); 
  Serial.print(", Position: "); 
  Serial.println(Pos, BIN); 
}

//--------------------------------------------------------------------------------------------
void notifyCVInfo(uint8_t State ) {
  Serial.print("CV Prog STATE: "); 
  Serial.println(State); 
}

//--------------------------------------------------------------------------------------------
void notifyCVResult(uint8_t cvAdr, uint8_t cvData ) {
  Serial.print("CV Prog Read: "); 
  Serial.print(cvAdr); 
  Serial.print(", "); 
  Serial.println(cvData); 
}
