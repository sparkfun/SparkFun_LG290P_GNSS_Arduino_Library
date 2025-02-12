/*
  General information about the attached device.
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

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
  Serial.println("SparkFun LG290P Device Info example");
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

  bool gpsEnabled, glonassEnabled, galileoEnabled, bdsEnabled, qzssEnabled, navicEnabled;
  Serial.print("Constellations: ");
  if (myGNSS.getConstellations(gpsEnabled, glonassEnabled, galileoEnabled, bdsEnabled, qzssEnabled, navicEnabled))
  {
    Serial.printf("GPS: %s, GLONASS: %s, Galileo: %s, Beidou: %s, QZSS: %s, NavIC: %s\r\n", 
      gpsEnabled ? "Yes" : "No", glonassEnabled ? "Yes" : "No", galileoEnabled ? "Yes" : "No", 
      bdsEnabled ? "Yes" : "No", qzssEnabled ? "Yes" : "No", navicEnabled ? "Yes" : "No");
  }
  else
  {
    Serial.println("Unavailable");
  }

  std::string serial;
  if (myGNSS.getSerialNumber(serial))
  {
    Serial.printf("Serial number: %s\r\n", serial.c_str());
  }
  else
  {
    Serial.printf("Serial number unavailable");
  }

  std::string version, buildDate, buildTime;
  if (myGNSS.getVersionInfo(version, buildDate, buildTime))
  {
    Serial.printf("Version info: %s %s %s\r\n", version.c_str(), buildDate.c_str(), buildTime.c_str());
  }
  else
  {
    Serial.printf("Version info unavailable");
  }

  uint32_t baud;
  uint8_t dataBits, parity, stop, flowControl;
  for (int i=1; i<=3; ++i)
  {
    Serial.printf("Port %d: ", i);
    if (myGNSS.getPortInfo(i, baud, dataBits, parity, stop, flowControl))
      Serial.printf("Baud=%lu Data bits=%d Parity=%d Stop=%d FlowControl=%d\r\n", baud, dataBits, parity, stop, flowControl);
    else
      Serial.println("Unknown");
  }

  bool ppsEnabled, alwaysOutput, positivePolarity;
  uint16_t duration;
  if (myGNSS.getPPS(ppsEnabled, duration, alwaysOutput, positivePolarity))
  {
    Serial.printf("PPS=%s, Duration=%dms, Always=%s, Polarity=%s\r\n",
      ppsEnabled ? "Enabled" : "Disabled", duration, alwaysOutput ? "Yes" : "No", positivePolarity ? "Pos" : "Neg");
  }
  else
  {
    Serial.printf("PPS: Not available\r\n");
  }

  int mode;
  if (myGNSS.getMode(mode))
  {
    Serial.printf("Current mode: %s\r\n", mode == 2 ? "BASE" : "ROVER");
  }

  const char * const msgs[] = {"GGA", "RMC", "PQTMEPE", "PQTMPL", "PQTMPVT", "PQTMSVINSTATUS" };
  const int vers[] = { -1, -1, 2, 1, 1, 1 };
  Serial.printf("Message rates: ");
  for (int i=0; i<6; ++i)
  {
    int rate;
    if (myGNSS.getMessageRate(msgs[i], rate, vers[i]))
      Serial.printf("%s: %d  ", msgs[i], rate);
  }
  Serial.println();
}

void loop()
{
}
