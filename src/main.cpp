
/*
   rocrail_CAN_GW

    Pogramme permettant le pilotage de locomotives à partir de Rocrail©
    et la mise a jour de la liste des locomotives en MFX.


*/

#define PROJECT "rocrail_can_gateway"
#define VERSION "0.2"
#define AUTHOR "Christophe BOBILLE : christophe.bobille@gmail.com"

//----------------------------------------------------------------------------------------
//  Board Check
//----------------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif


//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------

#include <ACAN_ESP32.h>
#include <Arduino.h>

//----------------------------------------------------------------------------------------
//   Debug serial
//----------------------------------------------------------------------------------------

#define RXD1 13
#define TXD1 14
#define debug Serial1

//----------------------------------------------------------------------------------------
//  CAN Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 250UL * 1000UL; // Marklin CAN baudrate = 250Kbit/s


// Buffers  : Rocrail always send 13 bytes
char cBuffer[13]; // CAN buffer
byte sBuffer[13]; // Serial buffer

uint16_t RrHash; // for Rocrail hash


//#include <WiFi.h>
// WiFiClient localClient;
// const char *ssid = "**********";
// const char *password = "***********";
// const uint port = 15731;
// const char *ip = "192.168.1.13"; // tcpbin.com's ip

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup()

{
  //--- Start serial
  Serial.begin(500000); // Rocrail baudrate to ESP32 USB
  Serial.setTimeout(2);

    while (!Serial)
  {
    debug.print(".");
    delay(200);
  }

  delay(100);
  debug.begin(115200, SERIAL_8N1, RXD1, TXD1); // For debug
  delay(100);

  debug.printf("\nProject   :    %s", PROJECT);
  debug.printf("\nVersion   :    %s", VERSION);
  debug.printf("\nAuteur    :    %s", AUTHOR);
  debug.printf("\nFichier   :    %s", __FILE__);
  debug.printf("\nCompiled  :    %s", __DATE__);
  debug.printf(" - %s\n\n", __TIME__);

  debug.println("Start setup");

  //--- Configure ESP32 CAN

  debug.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mRxPin = GPIO_NUM_22; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_23; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);

  if (errorCode)
  {
    debug.print("Configuration error 0x");
    debug.println(errorCode, HEX);
  }
  else
    debug.println("Configuration CAN OK");

  uint8_t rb = 0;
  while (rb != 13)
  {
    if (Serial.available())
      rb = Serial.readBytes(cBuffer, 13);
  }

  debug.println("");

  // extract the Rocrail hash
  RrHash = ((cBuffer[2] << 8) | cBuffer[3]);
  
  debug.printf("Rocrail hash : 0x");
  debug.println(RrHash, HEX);

  // --- register Rocrail on the CAN bus
  
  CANMessage frame;
  frame.id = (cBuffer[0] << 24) | (cBuffer[1] << 16) | RrHash;
  frame.ext = true;
  frame.len = cBuffer[4];
  for (byte i = 0; i < frame.len; i++)
    frame.data[i] = cBuffer[i + 5];
  const uint32_t ok = ACAN_ESP32::can.tryToSend(frame);

  debug.printf("frame.id : %d\n", frame.id);
  debug.print("frame.id hex : ");
  debug.print(frame.id, HEX);
  debug.print("len : ");
  debug.print(frame.len);
}



//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop()
{

  CANMessage frameIn;
  if (ACAN_ESP32::can.receive(frameIn))
  {
    // clear sBuffer
    for (byte el : sBuffer)
      el = 0;

    debug.printf("Reception CAN -> envoi Serial\n");
    debug.print("frame.id = 0x");
    debug.println(frameIn.id);

    sBuffer[0] = (frameIn.id & 0xFF000000) >> 24;
    sBuffer[1] = (frameIn.id & 0xFF0000) >> 16;
    sBuffer[2] = (frameIn.id & 0xFF00) >> 8; // hash
    sBuffer[3] = (frameIn.id & 0x00FF);      // hash
    sBuffer[4] = frameIn.len;
    for (byte i = 0; i < frameIn.len; i++)
      sBuffer[i + 5] = frameIn.data[i];
      
    Serial.write(sBuffer, 13);
  }

  // clear cBuffer
  for (byte el : cBuffer)
    el = 0;


  if (Serial.available())
  {
    debug.printf("Reception Serial -> envoi CAN\n");
    if (Serial.readBytes(cBuffer, 13) == 13)
    {
      debug.printf("Reception Serial ok\n");
      CANMessage frameOut;
      frameOut.id = (cBuffer[0] << 24) | (cBuffer[1] << 16) | RrHash;
      frameOut.ext = true;
      frameOut.len = cBuffer[4];
      for (byte i = 0; i < frameOut.len; i++)
        frameOut.data[i] = cBuffer[i + 5];
      const bool ok = ACAN_ESP32::can.tryToSend(frameOut);
    }
  }
}

//----------------------------------------------------------------------------------------