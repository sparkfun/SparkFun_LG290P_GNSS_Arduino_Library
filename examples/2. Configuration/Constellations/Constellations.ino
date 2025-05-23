/*
  Displaying satellites in view
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to enable and disable the various satellite constellations.
  This sketch is similar to Satellites.ino.

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
  Serial.println("SparkFun Constellations example");
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

  myGNSS.setMessageRate("GSV", 1);

  // This example will only work if the device is in ROVER mode
  myGNSS.ensureModeRover();
}

void loop()
{
  Serial.println();
  Serial.println("Test 1: All constellations enabled");
  myGNSS.setConstellations(true, true, true, true, true, true);
  busyWait(60);

  Serial.println();
  Serial.println("Test 2: Only GP, GA, GQ constellations enabled");
  myGNSS.setConstellations(true, false, true, false, true, false);
  busyWait(60);

  Serial.println();
  Serial.println("Test 3: Only GL, GB, IN constellations enabled");
  myGNSS.setConstellations(false, true, false, true, false, true);
  busyWait(60);
}

void busyWait(int nsecs)
{
  for (unsigned long start = millis(); millis() - start < 1000 * nsecs;)
  {
    myGNSS.update(); // Regularly call to parse any new data

    if (myGNSS.isNewSatelliteInfoAvailable())
    {
      auto sats = myGNSS.getVisibleSats();
      std::string prevTalker = "";
      Serial.printf("In view: %d ", sats.size());
      for (auto sat : sats)
      {
        if (sat.talker != prevTalker)
        {
          Serial.printf("  %s: ", sat.talker);
          prevTalker = sat.talker;
        }
        Serial.printf("%d ", sat.prn);
      }
      Serial.println();
    }
  }
}