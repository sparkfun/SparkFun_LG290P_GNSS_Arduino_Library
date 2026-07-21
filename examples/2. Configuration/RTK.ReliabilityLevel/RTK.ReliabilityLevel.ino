/*
  Configuring the RTK reliability level
  By: Paul Clark + Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 21 July 2026
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to configure the RTK reliability level.
  1 = Very relax, 2 = Relax, 3 = Medium, 4 = Strict, 5 = Very strict. Default is 3 on LG290P

  The LG290P firmware must be v1.06 ("LG290P03AANR01A06S") or higher for these commands to work. We recommend using the QGNSS software
  to update the LG290P firmware.
  Latest Firmware: https://github.com/sparkfun/SparkFun_RTK_Postcard/tree/main/Firmware
  Docs: https://docs.sparkfun.com/SparkFun_LG290P_Quadband_GNSS_RTK_Breakout/software_overview/#qgnss-software

  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Quadband GNSS RTK Breakout - LG290P (GPS-26620) https://www.sparkfun.com/products/26620

  Hardware Connections:
  Connect RX3 (green wire) of the LG290P to pin 17 on the ESP32
  Connect TX3 (orange wire) of the LG290P to pin 16 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a multi-band GNSS antenna: https://www.sparkfun.com/products/21801
*/

#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

// Adjust these values according to your configuration
//------------------------------------------------------------------------------
// https://www.sparkfun.com/sparkfun-gnss-flex-phat.html
#ifdef ESP32_RPI_FLEX

// ESP32 WROOM with 40-pin Raspberry Pi GPIO connector
// https://copperhilltech.com/esp32-development-board-with-raspberry-pi-gpio/

// UART1_TX (IO15) --> RPi GPIO Connector 10 (GPIO15/TXD) --> Raspberry Pi Flex Hat J4-19 (RXD1) --> Flex connector J3-19 (RXD1) --> LG290P 21 (RXD)
//int pin_UART1_TX = 15;

// UART1_RX (IO14) <-- RPi GPIO Connector 8 (GPIO14/RXD) <-- Raspberry Pi Flex Hat J4-15 (TXD1) <-- Flex connector J3-15 (TXD1) <-- LG290P 20 (TXD1)
//int pin_UART1_RX = 14;

// UART1_TX (IO5/CE0) --> RPi GPIO Connector 24 (GPIO8/CE0) --> Raspberry Pi Flex Hat J4-12 (RXD2) --> Flex connector J3-12 (RXD2) --> LG290P 7 (RXD)
int pin_UART1_TX = 5;

// UART1_RX (IO19/MISO) <-- RPi GPIO Connector 21 (GPIO9/MISO) <-- Raspberry Pi Flex Hat J4-10 (TXD2) <-- Flex connector J3-10 (TXD2) <-- LG290P 6 (TXD1)
int pin_UART1_RX = 19;

// Reset                                           ___                            _____               _____
//  No connection --> Raspberry Pi Flex Hat J4-16 (RST) --> Flex connector J3-16 (RESET)--> LG290P 8 (Reset)
int pin_RESET = -1;

const char * platform = "ESPBERRY & SparkFun GNSS Flex pHAT";

#else  // ESP32_RPI_FLEX
#ifdef  POSTCARD

// https://www.sparkfun.com/sparkfun-rtk-postcard.html
int pin_UART1_TX = 22;
int pin_UART1_RX = 21;
int pin_RESET = 33;
const char * platform = "SparkFun RTK Postcard";

#else   // POSTCARD

// SparkFun Thing Plus - ESP32 WROOM (USB-C)
// https://www.sparkfun.com/sparkfun-thing-plus-esp32-wroom-usb-c.html
// Board: SparkFun ESP32 Thing Plus C
int pin_UART1_TX = 17;
int pin_UART1_RX = 16;
int pin_RESET = -1;
const char * platform = "SparkFun ESP32 Thing Plus C";

#endif  // POSTCARD
#endif  // ESP32_RPI_FLEX

//------------------------------------------------------------------------------

int gnss_baud = 460800;

LG290P myGNSS;
HardwareSerial SerialGNSS(1); // Use UART1 on the ESP32

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun RTK reliability level example");

  // Issue the reset
  if (pin_RESET != -1)
  {
    Serial.println("Resetting the LG290P");
    pinMode(pin_RESET, OUTPUT);
    digitalWrite(pin_RESET, 0);
    delay(100);
    digitalWrite(pin_RESET, 1);
  }

  Serial.println("Initializing device...");

  // We must start the serial port before using it in the library
  // Increase buffer size to handle high baud rate streams
  SerialGNSS.setRxBufferSize(1024);
  SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  // myGNSS.enableDebugging(Serial); // Print all debug to Serial
  if (myGNSS.begin(SerialGNSS, "SFE_LG290P_GNSS_Library", output) == false)     // Give the serial port over to the library
  {
    Serial.println("LG290P failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("LG290P detected!");

  Serial.println("Ensuring ROVER mode");
  myGNSS.ensureModeRover();

  // Check firmware version and print info
  int versionMajor;
  int versionMinor;
  int versionCombined;
  myGNSS.getFirmwareVersionMajor(versionMajor);
  myGNSS.getFirmwareVersionMinor(versionMinor);
  myGNSS.getFirmwareVersion(versionCombined); // v1.06 becomes 106
  Serial.printf("Firmware v%d.%d (%d)\r\n", versionMajor, versionMinor, versionCombined);
  if (versionCombined < 106)
    Serial.println("Warning! get/setRtkReliabilityLevel requires firmware >= 1.06");

  // level : 1 = Very relax, 2 = Relax, 3 = Medium, 4 = Strict, 5 = Very strict
  uint16_t level = 3;
  if (myGNSS.getRtkReliabilityLevel(level) == false)
  {
    Serial.println("Failed to read reliability level. Do you have version 1.06 or newer of the LG290P firmware installed?");
  }
  else
  {
    Serial.printf("Successfully checked reliability level: %d (%s)\r\n", level,
                  level == 1 ? "Very relax" : level == 2 ? "Relax" :
                  level == 3 ? "Medium" : level == 4 ? "Strict" :
                  level == 5 ? "Very strict" : "Undefined");

    // Try setting to Relax
    if (myGNSS.setRtkReliabilityLevel(2) == false)
      Serial.println("Failed to set reliability level");
    else
    {
      if (myGNSS.getRtkReliabilityLevel(level) == false)
        Serial.println("Failed to read reliability level");
      else
      {
        Serial.printf("Reliability level set to %d (%s)\r\n", level,
                  level == 1 ? "Very relax" : level == 2 ? "Relax" :
                  level == 3 ? "Medium" : level == 4 ? "Strict" :
                  level == 5 ? "Very strict" : "Undefined");
      }
    }
  }
}

void loop()
{

}

//----------------------------------------
// Output a buffer of data
//
// Inputs:
//   buffer: Address of a buffer of data to output
//   length: Number of bytes of data to output
//----------------------------------------
void output(uint8_t * buffer, size_t length)
{
    size_t bytesWritten;

    if (Serial)
    {
        while (length)
        {
            // Wait until space is available in the FIFO
            while (Serial.availableForWrite() == 0);

            // Output the character
            bytesWritten = Serial.write(buffer, length);
            buffer += bytesWritten;
            length -= bytesWritten;
        }
    }
}
