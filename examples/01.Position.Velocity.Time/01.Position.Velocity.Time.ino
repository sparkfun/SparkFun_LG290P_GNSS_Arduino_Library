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
  myGNSS.setMessageRate("GGA", 0);
  myGNSS.setFixInterval(1000);
}

void loop()
{
  myGNSS.update(); // Regularly call to parse any new data

  if (millis() - lastCheck > 1000)
  {
    lastCheck = millis();

    // The 'get' methods are updated whenever new data is parsed with the update() call.
    // By default, this data is updated once per second.
    Serial.printf("Lat/Long/Alt: %.8f/%.8f/%.2f\r\n", myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude());
    Serial.printf("Horizontal Speed: %.2f m/s  Course: %.2f degrees\r\n",
                  myGNSS.getHorizontalSpeed(), myGNSS.getCourse());
    Serial.printf("Date (yyyy/mm/dd): %04d/%02d/%02d Time (hh:mm:ss) %02d:%02d:%02d.%03d Leap Seconds: %d\r\n",
                  myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(),
                  myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getMillisecond(),
                  myGNSS.getLeapSecond());
    Serial.printf("Satellites in view: %d\r\n", myGNSS.getSatellitesInViewCount());
    Serial.printf("Fix quality: %d - ", myGNSS.getFixQuality());
    switch (myGNSS.getFixQuality())
    {
      default:
        Serial.println("Unknown");
        break;
      case 0:
        Serial.println("No fix");
        break;
      case 1:
        Serial.println("3D Fix");
        break;
      case 2:
        Serial.println("DGPS Fix");
        break;
      case 3:
        Serial.println("GPS PPS Mode, fix valid");
        break;
      case 4:
        Serial.println("RTK Fix");
        break;
      case 5:
        Serial.println("RTK Float");
        break;
    }
    Serial.printf("HDOP: %.2f  PDOP: %.2f\r\n", myGNSS.getHdop(), myGNSS.getPdop());
    Serial.printf("Time of Week: %lu\r\n", myGNSS.getTimeOfWeek());
    Serial.printf("Geoidal Separation: %.2f\r\n", myGNSS.getGeoidalSeparation());
    Serial.printf("Velocity (N/E/D): (%.2f,%.2f,%.2f)\r\n", myGNSS.getNorthVelocity(), myGNSS.getEastVelocity(), myGNSS.getDownVelocity());
    Serial.println();
  }
}