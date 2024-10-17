/*
  Get and sent the PPS signal line characteristics
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

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
  Serial.println("SparkFun LG290P PPS (pulse per second) example");
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
  Serial.println("Watch the PPS LED flash.");
  Serial.println();
}

void loop()
{
  for (auto duration : {20, 100, 600})
    for (auto polarity : {true, false})
    {
        Serial.printf("PPS set to %d ms, %s polarity\r\n", duration, polarity ? "POSITIVE" : "NEGATIVE");
        if (!myGNSS.enablePPS(duration, true, polarity))
            Serial.println("Error!");

        // Stay in this state for 10 seconds
        for (unsigned long start = millis(); millis() - start < 10000; )
            myGNSS.update();
    }

    Serial.printf("PPS disabled completely.\r\n");
    if (!myGNSS.disablePPS())
        Serial.println("Error!");

    // Stay in this state for 10 seconds
    for (unsigned long start = millis(); millis() - start < 10000; )
        myGNSS.update();
}

