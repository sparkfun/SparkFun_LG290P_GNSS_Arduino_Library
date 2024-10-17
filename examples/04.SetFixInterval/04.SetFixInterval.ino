/*
  Set device "Fix" speed
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to query a LG290P GNSS module for its position and time data.
  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  Note: Lat/Lon are doubles and the LG290P outputs 8 digits after the decimal.

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
  Serial.println("SparkFun LG290P Fix Rate example");
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
    static bool fast = true;
    int fixInterval = fast ? 100 : 1000;
    
    Serial.println();
    Serial.printf("We'll change the fix rate to '%s' (%dms) and do a hot restart\r\n", fast ? "fast" : "normal", fixInterval);
    busy_wait(2, false);

    Serial.printf("Changing the fix interval to %dms\r\n", fixInterval);
    Serial.println(myGNSS.setFixInterval(fixInterval) ? "Success!" : "Fail");
    busy_wait(2, false);

    Serial.printf("Performing hot reset\r\n");
    if (myGNSS.hotReset())
        Serial.println("Success!");
    busy_wait(2, false);
    busy_wait(30, true);

    fast = !fast;
}
