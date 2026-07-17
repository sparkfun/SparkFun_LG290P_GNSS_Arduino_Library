/*
  Set PQTMCFGNAVMODE
  By: Paul Clark, Nathan Seidle and Mikal Hart
  Date: July 17, 2026
  License: Public domain - do whatever you'd like.

  This example shows how to set/get the navigation mode using PQTMCFGNAVMODE.
  Note: requires LG290P firmware >= v2.01

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  LG290P Breakout: https://www.sparkfun.com/sparkfun-quadband-gnss-rtk-breakout-lg290p-qwiic.html
  LG290P PiHat: https://www.sparkfun.com/sparkfun-gnss-flex-phat-lg290p.html

  Hardware Connections:
  Connect RX3 (green wire) of the LG290P to pin 14 on the ESP32
  Connect TX3 (orange wire) of the LG290P to pin 13 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a multi-band GNSS antenna: https://www.sparkfun.com/products/21801
*/

#include <SparkFun_LG290P_GNSS.h> // Click here to get the library: http://librarymanager/All#SparkFun_LG290P

// Adjust these values according to your configuration
//------------------------------------------------------------------------------

// Pre-defined boards:
// #define ESP32_RPI_FLEX
// #define POSTCARD
#define ESP32_THING_PLUS_C

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
#ifdef ESP32_THING_PLUS_C

// https://www.sparkfun.com/sparkfun-thing-plus-esp32-wroom-usb-c.html
int pin_UART1_TX = 17;
int pin_UART1_RX = 16;
int pin_RESET = 4; // The Free pin
const char * platform = "SparkFun ESP32 Thing Plus C";

#else   // ESP32_THING_PLUS_C

// ???
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int pin_RESET = -1;
const char * platform = "???";

#endif  // ESP32_THING_PLUS_C
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
  Serial.println("SparkFun LG290P NAVMODE example");

  // Issue the reset - if pin_RESET is defined
  if (pin_RESET != -1)
  {
    Serial.println("Resetting the LG290P");
    pinMode(pin_RESET, OUTPUT);
    digitalWrite(pin_RESET, 0);
    delay(100);
    digitalWrite(pin_RESET, 1);
  }

  Serial.println("Initializing device...");

  // Increase buffer size to handle high baud rate streams
  // SerialGNSS.setRxBufferSize(1024 * 4);

  SerialGNSS.begin(gnss_baud, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  // myGNSS.enableDebugging(Serial);        // Print all debug to Serial
  if (myGNSS.begin(SerialGNSS, "SFE_LG290P_GNSS_Library", output) == false) // Give the serial port over to the library
  {
    Serial.println("LG290P failed to respond. Check ports and baud rates. Freezing...");
    while (true)
      ;
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
  myGNSS.getFirmwareVersion(versionCombined); // v2.01 becomes 201
  Serial.printf("Firmware v%d.%d (%d)\r\n", versionMajor, versionMinor, versionCombined);
  if (versionCombined < 202)
    Serial.println("Warning! get/setNavMode requires firmware >= 2.01");

  // Get/Set the current navigation mode
  // 0 = Normal mode. (Basic mode applied to most scenarios, for example, driving scenario)
  // 5 = Dynamic flight mode (applied to Dynamic flight mode with equivalent dynamics range
  //     and vertical acceleration on different flight phase)
  // 11 = Mower mode (applied to mower application) (*** Default value on LG290P ***)
  // 14 = Agriculture mode (applied to agriculture application)

  uint16_t mode;
  if (myGNSS.getNavMode(mode) == true)
    printNavMode(mode);
  else
    Serial.println("Failed to get the navigation mode. Is your LG290P firmware >= 2.01?");

  mode = 0;
  Serial.print("Setting navigation mode to ");
  Serial.println(mode);
  if (myGNSS.setNavMode(mode) == true)
    Serial.println("Set navigation mode: success");
  else
    Serial.println("Set navigation mode: failed!");

  if (myGNSS.getNavMode(mode) == true)
    printNavMode(mode);
  else
    Serial.println("Failed to get the navigation mode. Is your LG290P firmware >= 2.01?");
}

void loop()
{
  // Nothing to do here...
}

//----------------------------------------
// Print the navigation mode
//
// Inputs:
//   mode: Reference to a uint16_t containing the mode
//----------------------------------------
void printNavMode(uint16_t &mode)
{
  if (Serial)
  {
    Serial.print("Navigation mode=");
    Serial.print(mode);
    switch(mode)
    {
      default:
        Serial.println(", Reserved / undefined");
        break;
      case 0:
        Serial.println(", Normal mode");
        break;
      case 5:
        Serial.println(", Dynamic flight mode");
        break;
      case 11:
        Serial.println(", Mower mode (default for LG290P)");
        break;
      case 14:
        Serial.println(", Agriculture mode");
        break;
    }
  }
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
