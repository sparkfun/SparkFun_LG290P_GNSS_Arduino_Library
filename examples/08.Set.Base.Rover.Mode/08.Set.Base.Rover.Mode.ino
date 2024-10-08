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
  SparkFun Quadband GNSS RTK Breakout - LG290P (GPS-XXXXX) https://www.sparkfun.com/products/XXXX

  Hardware Connections:
  Connect RX3 (green wire) of the LG290P to pin 14 on the ESP32
  Connect TX3 (orange wire) of the LG290P to pin 13 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a multi-band GNSS antenna: https://www.sparkfun.com/products/21801
*/

#include <SparkFun_LG290P_GNSS.h>

// Adjust these values according to your configuration
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int gnss_baud = 460800;

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

void myNmeaCallback(NmeaPacket &packet)
{
  Serial.printf("Received an NMEA $%s%s packet: %s ...\r\n", packet.TalkerId().c_str(), packet.SentenceId().c_str(), packet[1].c_str());
}

void myRtcmCallback(RtcmPacket &packet)
{
  Serial.printf("Received an RTCM %d packet: ", packet.type);
  for (int i=0; i<packet.bufferlen && i<10; ++i)
    Serial.printf("%02X ", packet.buffer[i]);
  Serial.println(packet.bufferlen > 10 ? "..." : "");
}

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun LG290P Base/Rover Mode Example");

  // We must start the serial port before using it in the library
  // Increase buffer size to handle high baud rate streams
  SerialGNSS.setRxBufferSize(1024);
  SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);
  
  // myGNSS.enableDebugging(Serial); // Print all debug to Serial
  // if (!myGNSS.begin(SerialGNSS, &Serial, &Serial)) //Give the serial port over to the library
  if (!myGNSS.begin(SerialGNSS)) //Give the serial port over to the library
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


void busy_wait(int seconds)
{
    for (unsigned long start = millis(); millis() - start < 1000 * seconds; )
       myGNSS.update();
}

void loop()
{
    Serial.println();
    Serial.println("Here's the engine running in ROVER mode");
    myGNSS.setModeRover();
    myGNSS.saveParameters();
    myGNSS.factoryReset();
    Serial.println("Resetting device...");
    busy_wait(30);
    
    Serial.println();
    Serial.println("Here's the engine running in BASE mode");
    myGNSS.setModeBase();
    myGNSS.saveParameters();
    myGNSS.factoryReset();
    Serial.println("Resetting device...");
    busy_wait(30);
}
