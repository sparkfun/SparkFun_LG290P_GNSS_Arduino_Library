/*
  Displaying satellites in view
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how enable the various "survey in" modes.  

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
  delay(2000);
  Serial.println();
  Serial.println("SparkFun Survey In Mode Example");

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

  Serial.println("Subscribing to PQTMSVINSTATUS message");
  myGNSS.nmeaSubscribe("PQTMSVINSTATUS", MyCallback);

  Serial.println("Subscribing to RTCM #1005 message");
  myGNSS.rtcmSubscribe(1005, MyRtmcCallback);

  Serial.println("Setting base station mode");
  myGNSS.setModeBase();

  Serial.println("Enabling PQTMSVINSTATUS message");
  myGNSS.setMessageRate("PQTMSVINSTATUS", 1, 1);

  Serial.println("Setting 'Survey In' Mode");
  int secs = 20;
  Serial.printf("Give the device %d seconds to establish location.\r\n", secs);
  myGNSS.setSurveyInMode(secs);
  myGNSS.saveParameters();
  myGNSS.softwareReset();
  Serial.print("Waiting until device is back online... ");
  if (!myGNSS.isConnected())
  {
    Serial.println("reconnection failed; halting");
    while (true);
  }
  Serial.println();
  Serial.println("Online. Waiting for PQTMSVINSTATUS messages...");
}

void MyRtmcCallback(RtcmPacket &rtmc)
{
  if (rtmc.type == 1005)
  {
    Serial.printf("1005 Payload: ");
    for (int i=0; i<rtmc.payloadLen; ++i)
      Serial.printf("%02X ", rtmc.buffer[3 + i]);
    Serial.println();
    #if false
    byte *payload = rtmc.buffer + 3;
    int64_t x = *(int64_t *)(payload + 5);
    int64_t y = *(int64_t *)(payload + 10);
    int64_t z = *(int64_t *)(payload + 10);
    x &= 0x3FFFFFFFFF;
    y &= 0x3FFFFFFFFF;
    z &= 0x3FFFFFFFFF;
    Serial.printf("XYZ=(%lld,%lld,%lld)\r\n", x, y, z);
    #endif
  }
}

void MyCallback(NmeaPacket &nmea)
{
  std::string tow = nmea[2];
  std::string validity = nmea[3] == "0" ? "Invalid" : nmea[3] == "1" ? "In-progress" : "Valid";
  std::string posCount = nmea[6];
  std::string total = nmea[7];
  std::string ecefX = nmea[8];
  std::string ecefY = nmea[9];
  std::string ecefZ = nmea[10];
  std::string accuracy = nmea[11];

  Serial.printf("Week time: %s  Validity: '%s'  Pos: %s/%s  ECEF: (%s,%s,%s)  Accuracy: %sm\r\n",
    tow.c_str(), validity.c_str(), posCount.c_str(), total.c_str(), ecefX.c_str(), 
    ecefY.c_str(), ecefZ.c_str(), accuracy.c_str());
}

void loop()
{
  myGNSS.update(); // Regularly call to parse any new data
}
