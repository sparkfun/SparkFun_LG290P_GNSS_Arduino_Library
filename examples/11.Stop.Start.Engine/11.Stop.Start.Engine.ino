/*
  Stop and start the GNSS engine
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to stop and start the GNSS engine.

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
  Serial.println("SparkFun LG290P Fix Rate Example");

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
}

void busy_wait(int seconds, bool printinfo)
{
    for (unsigned long start = millis(); millis() - start < 1000 * seconds; )
    {
        if (printinfo && myGNSS.isNewSnapshotAvailable())
            Serial.printf("%02d:%02d:%02d.%03d Lat/Long=(%.8f,%.8f) Alt=%.2f\r\n", 
            myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getMillisecond(),
            myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude());
        myGNSS.update();
    }
}

void loop()
{
    myGNSS.enableEngine();
    Serial.println();
    Serial.println("Here's the engine running normally with $PQTMGNSSSTART");
    busy_wait(2, false);
    busy_wait(15, true);
    
    myGNSS.disableEngine();
    Serial.println();
    Serial.println("Here's the engine shut off with $PQTMGNSSSTOP");
    busy_wait(2, false);
    busy_wait(15, true);
}
