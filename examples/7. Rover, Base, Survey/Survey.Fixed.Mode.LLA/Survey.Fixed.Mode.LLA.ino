/*
  Setting "Survey Fixed" Mode
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how enable the fixed survey mode, where you define the BASE station's
  location by supplying setSurveyFixedMode with Latitude/Longitude/Altitude coordinates.

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
int mode;

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun Survey In Mode example");
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

  // You MUST be in BASE mode to set Survey In Mode
  Serial.println("Setting base station mode");
  verify(myGNSS.setModeBase(false)); // don't reset, because setSurveyInMode() is going to do it
  
  double eiffelLat = 48.8584, eiffelLong = 2.2945, eiffelAlt = 365;
  double ecefX, ecefY, ecefZ;

  // Convert Eiffel Tower lat/long/alt to ECEF coordinates
  LG290P::geodeticToEcef(eiffelLat, eiffelLong, eiffelAlt, ecefX, ecefY, ecefZ);

  Serial.println("Setting 'Survey' Mode fixed to top of Eiffel Tower");
  Serial.printf("(%.3f,%.3f,%.3f)\r\n", ecefX, ecefY, ecefZ);
  verify(myGNSS.setSurveyFixedMode(ecefX, ecefY, ecefZ));
  if (!myGNSS.isConnected())
  {
    Serial.println("reconnection failed; halting");
    while (true);
  }
  Serial.println("Online!");

  verify(myGNSS.getMode(mode));
}

unsigned long lastUpdate = 0;

void loop()
{
  myGNSS.update(); // Regularly call to parse any new data
  if (millis() - lastUpdate >= 1000)
  {
    lastUpdate = millis();
    int svinStatus = myGNSS.getSurveyInStatus();
    const char *status = svinStatus == 1 ? "In Progress" : svinStatus == 2 ? "Valid" : "Unknown/Invalid";
    Serial.printf("%02d:%02d:%02d: Mode = '%s' Status = '%s' (%d/%d) (%.4f,%.4f,%.4f) Mean Accuracy = %.4f\r\n",
      myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), mode == 2 ? "BASE" : "ROVER", status,
      myGNSS.getSurveyInObservations(), myGNSS.getSurveyInCfgDuration(),
      myGNSS.getSurveyInMeanX(), myGNSS.getSurveyInMeanY(), myGNSS.getSurveyInMeanZ(), 
      myGNSS.getSurveyInMeanAccuracy());
  }
}

void verify(bool event)
{
  if (!event)
  {
    Serial.println("Operation failed: freezing!");
    while (true);
  }
}
