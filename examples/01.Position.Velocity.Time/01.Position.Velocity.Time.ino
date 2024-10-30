/*
  Reading Position and Time via Serial
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

unsigned long lastCheck = 0;

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();

  Serial.println("SparkFun LG290P Position, Velocity, Time example");
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
}

void loop()
{
  myGNSS.update(); // Regularly call to parse any new data

  if (millis() - lastCheck > 1000)
  {
    lastCheck = millis();

    static int linecount = 0;
    if (linecount++ % 20 == 0)
    {
      // Every 20th line draw the helpful header
      const char *headings[] = { "Date", "Time", "Latitude", "Longitude", "Altitude", "Speed", "North", "East", "Down", "Sat", "SIV", "Fix-Quality", "HDOP", "PDOP", "Leap", "Sep" };
      int widths[] = {      10,     8,      12,         13,          8,          7,       7,       7,      7,      3,     3,     11,            5,      5,     4,       7    };
      int items = sizeof widths / sizeof widths[0];
      Serial.println();

      // Header
      for (int i=0; i<items; ++i)
      {
        char buf[10]; sprintf(buf, "%%-%ds ", widths[i]);
        Serial.printf(buf, headings[i]);
      }
      Serial.println();

      // Dashes
      for (int i=0; i<items; ++i)
      {
        std::string dashes(widths[i], '-');
        Serial.printf("%s%s", dashes.c_str(), i == items - 1 ? "" : "-");
      }
      Serial.println();
    }

    // Fix quality requires some special formatting
    char qualbuf[32];
    const char *qualities[] = { "No-Fix", "3D-Fix", "DGPS-Fix", "GPS-PPS", "RTK-Fix", "RTK-Flt" };
    int qual = myGNSS.getFixQuality();
    snprintf(qualbuf, sizeof qualbuf, "%s(%d)", (qual >= 0 && qual <= 5) ? qualities[qual] : "Unknown", qual);

    Serial.printf("%02d/%02d/%04d %02d:%02d:%02d %-12.8f %-13.8f %-8.2f %-7.2f %-7.2f %-7.2f %-7.2f %-3d %-3d %-11s %-5.2f %-5.2f %-4d %-7.2f\r\n",
      myGNSS.getDay(), myGNSS.getMonth(), myGNSS.getYear(),
      myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(),
      myGNSS.getLatitude(), myGNSS.getLongitude(),
      myGNSS.getAltitude(), myGNSS.getHorizontalSpeed(), myGNSS.getNorthVelocity(), myGNSS.getEastVelocity(), myGNSS.getDownVelocity(),
      myGNSS.getSatellitesUsedCount(), myGNSS.getSatellitesInViewCount(), qualbuf, myGNSS.getHdop(), myGNSS.getPdop(),
      myGNSS.getLeapSeconds(), myGNSS.getGeoidalSeparation());
  }
}

