/*
  Subscribing and unsubscribing to NMEA and RTCM messages
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to subscribe to NMEA sentences.

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

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun LG290P PQTM Message Enable Example");

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
  Serial.println();
  Serial.println("*** Normal mode ***");
}

void busy_wait(int secs)
{
  while (secs)
  {
    static unsigned long last = 0;
    if (millis() - last >= 1000)
    {
      Serial.printf("Lat/Long/Alt: %.8f/%.8f/%.2f\r\n", myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude());
      last = millis();
      --secs;
    }
    myGNSS.update(); // Regularly call to parse any new data
  }
}

void PVTCallback(NmeaPacket &nmea)
{
  Serial.printf("Found! '%s'\r\n", nmea.ToString().c_str());
}

void loop()
{
  busy_wait(10);
  Serial.println();
  Serial.println("*** Enable and subscribe to PVT and ODO sentences ***");
  myGNSS.nmeaSubscribe("PQTMPVT", PVTCallback);
  myGNSS.setMessageRate("PQTMPVT", 1, 1);
  myGNSS.nmeaSubscribe("PQTMODO", PVTCallback);
  myGNSS.setMessageRate("PQTMODO", 1, 1);
  busy_wait(10);
  Serial.println();
  Serial.println("*** Disable PVT and ODO sentences but continue subscribing ***");
  myGNSS.setMessageRate("PQTMPVT", 0, 1);
  myGNSS.setMessageRate("PQTMODO", 0, 1);
  busy_wait(10);
  Serial.println();
  Serial.println("*** Unsubscribe from PVT and ODO sentences too ***");
  myGNSS.nmeaUnsubscribe("PQTMPVT");
  myGNSS.nmeaUnsubscribe("PQTMODO");
}

