/*
  Put the GNSS receiver into BASE or ROVER mode
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to put the LG290P into "BASE" mode, where it transmits RTCM packets 
  instead of NMEA, then back to ROVER mode.

  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Quadband GNSS RTK Breakout - LG290P (GPS-26620) https://www.sparkfun.com/products/26620

  Hardware Connections:
  Connect RX3 (green wire) of the LG290P to pin 14 on the ESP32
  Connect TX3 (orange wire) of the LG290P to pin 13 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a multi-band GNSS antenna: https://www.sparkfun.com/products/21801
*/

#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

// Adjust these values according to your configuration
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int gnss_baud = 460800;

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun LG290P Base/Rover Mode example");
  Serial.println("Initializing device...");

  // We must start the serial port before using it in the library
  // Increase buffer size to handle high baud rate streams
  SerialGNSS.setRxBufferSize(1024);
  SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);
  
  // myGNSS.enableDebugging(Serial); // Print all debug to Serial
  if (myGNSS.begin(SerialGNSS) == false)     // Give the serial port over to the library
  {
    Serial.println("LG290P failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }

  Serial.println("LG290P detected!");

  myGNSS.nmeaSubscribe("GGA", myNmeaCallback);
  myGNSS.nmeaSubscribe("RMC", myNmeaCallback);
  myGNSS.rtcmSubscribe(1005, myRtcmCallback);
  myGNSS.rtcmSubscribe(1124, myRtcmCallback);
  myGNSS.rtcmSubscribe(1074, myRtcmCallback);
}

void loop()
{
    Serial.println();
    myGNSS.setModeRover();
    myGNSS.save();
    Serial.println("Resetting device...");
    Serial.println("Here's the engine running in ROVER mode");
    myGNSS.reset();
    monitorActivity(30);

    Serial.println();
    myGNSS.setModeBase();
    myGNSS.save();
    Serial.println("Resetting device...");
    Serial.println("Here's the engine running in BASE mode");
    myGNSS.reset();
    monitorActivity(30);
}

void myNmeaCallback(NmeaPacket &packet)
{
  Serial.printf("$%s%s ", packet.TalkerId().c_str(), packet.SentenceId().c_str());
}

void myRtcmCallback(RtcmPacket &packet)
{
  Serial.printf("RTCM-%d ", packet.type);
}

void monitorActivity(int seconds)
{
  if (myGNSS.isConnected())
  {
    int ggaRate = 0, rmcRate = 0, svinstatusRate = 0, pvtRate = 0, plRate = 0, epeRate = 0, rtcm1005Rate = 0, rtcm107XRate = 0;
    int mode = 0;
    myGNSS.getMessageRate("GGA", ggaRate);
    myGNSS.getMessageRate("RMC", rmcRate);
    myGNSS.getMessageRate("PQTMSVINSTATUS", svinstatusRate, 1);
    myGNSS.getMessageRate("PQTMPVT", pvtRate, 1);
    myGNSS.getMessageRate("PQTMPL", plRate, 1);
    myGNSS.getMessageRate("PQTMEPE", epeRate, 2);
    myGNSS.getMessageRate("RTCM3-1005", rtcm1005Rate);
    myGNSS.getMessageRate("RTCM3-107X", rtcm107XRate);

    myGNSS.getMode(mode);
    Serial.printf("Mode reported as '%s'\r\n", mode == 1 ? "ROVER" : "BASE");
    Serial.printf("Msgs enabled: GGA=%c RMC=%c, SVINSTATUS=%c, PVT=%c, PL=%c EPE=%c 1005=%c 107X=%c\r\n",
      ggaRate ? 'Y' : 'N', rmcRate ? 'Y' : 'N', svinstatusRate ? 'Y' : 'N', pvtRate ? 'Y' : 'N',
      plRate ? 'Y' : 'N', epeRate ? 'Y' : 'N', rtcm1005Rate ? 'Y' : 'N', rtcm107XRate ? 'Y' : 'N');
  }

  for (unsigned long start = millis(); millis() - start < 1000 * seconds; )
    myGNSS.update();
  Serial.println();
}