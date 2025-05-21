/*
  Configuring minimum elevation and CNR used in the positioning engine
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 20 May 2025
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to configure the min elevation and CNR used for the position engine.

  It is a known practice to raise the minimum elevation from the default of 5 degrees to 10 or even 15 degrees 
  when in an urban environment to exclude satellites that are too low and may be blocked by structures causing 
  multipath errors. Similarly, increasing the minimum CNR from 10 dBHz to 20 or 25 dBHz can exclude satellites
  with lower signal strength from being used in the position calculation.

  The LG290P firmware must be v5 ("LG290P03AANR01A05S") or higher for these commands to work. We recommend using the QGNSS software
  to update the LG290P firmware.
  Latest Firmware: https://github.com/sparkfun/SparkFun_RTK_Postcard/tree/main/Firmware
  Docs: https://docs.sparkfun.com/SparkFun_LG290P_Quadband_GNSS_RTK_Breakout/software_overview/#qgnss-software

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
  Serial.println("SparkFun Elevation and CNR example");
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

  int elevationAngle = 0;
  if (myGNSS.getElevationAngle(elevationAngle) == false)
  {
    Serial.println("Failed to read elevation angle. Do you have version 5 or newer of the LG290P firmware installed?");
  }
  else
  {
    Serial.printf("Successfully checked elevation angle: %d degrees\r\n", elevationAngle);
    
    // 5 is default meaning any satellite 5 degrees above the horizon or higher will be used
    // 90 to -90 is allowed
    if (myGNSS.setElevationAngle(10) == false)
      Serial.println("Failed to set elevation angle");
    else
    {
      if (myGNSS.getElevationAngle(elevationAngle) == false)
        Serial.println("Failed to read elevation angle.");
      else
      {
        Serial.printf("Elevation angle set to %d degrees.\r\n", elevationAngle);
      }
    }
  }

  float cnr = 0;
  if (myGNSS.getCNR(cnr) == false)
  {
    Serial.println("Failed to read CNR. Do you have version 5 or newer of the LG290P firmware installed?");
  }
  else
  {
    Serial.printf("Successfully checked CNR: %0.1f dBHz\r\n", cnr);
    
    // 10.0 is default meaning any satellite with greater than 10 dBHz signal level will be included in the position fix calculation
    // 0.0 to 99.0 dBHz is allowed
    if (myGNSS.setCNR(15.0) == false)
      Serial.println("Failed to set CNR");
    else
    {
      if (myGNSS.getCNR(cnr) == false)
        Serial.println("Failed to read CNR.");
      else
      {
        Serial.printf("CNR set to %0.1f dBHz.\r\n", cnr);
      }
    }
  }  
}

void loop()
{

}