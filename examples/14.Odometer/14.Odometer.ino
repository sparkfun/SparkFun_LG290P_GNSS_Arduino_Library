/*
  Playing with the LG290P Odometer feature
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to enable and use the onboard odometer.

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
  Serial.println("SparkFun LG290P Odometer example");
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
  Serial.println("*** Enable and subscribe to ODO sentences ***");
  myGNSS.nmeaSubscribe("PQTMODO", MyPqtmCallback);  
  myGNSS.setMessageRate("PQTMODO", 1, 1);
  myGNSS.sendCommand("$PQTMCFGODO", ",W,1,0"); // enable odo
  myGNSS.setFixInterval(1000);
}

void MyPqtmCallback(NmeaPacket &nmea)
{
    bool odoEnabled = nmea[3] == "1";
    Serial.printf("This device's odometer is %senabled and it has traveled %s meters\r\n", odoEnabled ? "" : "NOT ", nmea[4].c_str());
}

void loop()
{
    myGNSS.update(); // Regularly call to parse any new data
}

