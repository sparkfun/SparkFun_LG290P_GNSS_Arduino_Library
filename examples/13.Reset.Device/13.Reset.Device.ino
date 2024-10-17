/*
  Stop and start the GNSS engine
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to perform the various device resets (hot, warm, cold, factory)

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
  delay(3000);
  Serial.println();
  Serial.println("SparkFun LG290P Reset Device example");
  Serial.println("Initializing device...");

  // We must start the serial port before using it in the library
  // Increase buffer size to handle high baud rate streams
  SerialGNSS.setRxBufferSize(1024);
  SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);
  
  // myGNSS.enableDebugging(Serial); // Print all debug to Serial
  if (!myGNSS.begin(SerialGNSS))     // Give the serial port over to the library
  {
    Serial.println("LG290P failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("LG290P detected!");
  Serial.println();

  Serial.println("At startup");
  busy_wait();
}

void busy_wait()
{
  // Delay 2 seconds to allow the message to be seen
  for (unsigned long start = millis(); millis() - start < 1000 * 2; )
    myGNSS.update();

  // For at least 10 seconds, but no more than 60, display lat/long until we have a fix
  for (unsigned long start = millis(); millis() - start < 1000 * 60; )
  {
    myGNSS.update();
    if (myGNSS.isNewSnapshotAvailable())
    {
      Serial.printf("%02d:%02d:%02d.%03d Lat/Long=(%.8f,%.8f) Alt=%.2f\r\n", 
        myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getMillisecond(),
        myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude());
        myGNSS.update();
      if (start - millis() >= 10000 && myGNSS.getLatitude() != 0)
        break;
    }
  }
}

void loop()
{
  myGNSS.setFixInterval(1000);

  Serial.println("Hot reset");
  myGNSS.hotReset();
  busy_wait();

  Serial.println("Warm reset");
  myGNSS.warmReset();
  busy_wait();

  Serial.println("Cold reset");
  myGNSS.coldReset();
  busy_wait();

  Serial.println("Factory reset");
  myGNSS.softwareReset();
  busy_wait();
}
