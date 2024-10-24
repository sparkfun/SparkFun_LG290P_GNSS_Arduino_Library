/*
  Displaying satellites in view
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how enable the fixed survey in modes.

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
  Serial.println("SparkFun Survey Fixed Mode example");
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

  Serial.println("Subscribing to RTCM #1005 message");
  myGNSS.rtcmSubscribe(1005, MyRtmcCallback);

  Serial.println("Setting base station mode");
  myGNSS.setModeBase();

  Serial.println("Setting 'Survey' Mode fixed to top of Eiffel Tower");
  myGNSS.setSurveyFixedMode(4200944.016, 168364.025, 4780802.825);
  myGNSS.save();
  myGNSS.reset();
  Serial.print("Waiting until device is back online... ");
  if (!myGNSS.isConnected())
  {
    Serial.println("reconnection failed; halting");
    while (true);
  }
  Serial.println("Online!");
}


void loop()
{
  myGNSS.update(); // Regularly call to parse any new data
}

void MyRtmcCallback(RtcmPacket &rtcm)
{
  Serial.printf("Received an RTCM %d packet.  ECEF: (%.4f,%.4f,%.4f)\r\n", rtcm.type, myGNSS.getEcefX(), myGNSS.getEcefY(), myGNSS.getEcefZ());
}
