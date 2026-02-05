/*
  Put the GNSS receiver into BASE or ROVER mode
  By: Nathan Seidle + Mikal Hart
  SparkFun Electronics
  Date: 29 September 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to put the LG290P into "BASE" mode, where it transmits RTCM packets
  instead of NMEA, then back to ROVER mode.

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

// Reset                                                 ___                            _____               _____
//  No connection --> Raspberry Pi Flex Hat J4-16 (RST) --> Flex connector J3-16 (RESET)--> LG290P 8 (Reset)
int pin_RESET = -1;

const char * platform = "ESPBERRY & SparkFun GNSS Flex pHAT";

#else  // ESP32_RPI_FLEX
#ifdef  POSTCARD

// https://www.sparkfun.com/sparkfun-rtk-postcard.html
int pin_UART1_TX = 21;
int pin_UART1_RX = 22;
int pin_RESET = 33;
const char * platform = "SparkFun RTK Postcard";

#else   // POSTCARD

// ???
int pin_UART1_TX = 14;
int pin_UART1_RX = 13;
int pin_RESET = -1;
const char * platform = "???";

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
  Serial.println("SparkFun LG290P Base/Rover Mode example");

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

  myGNSS.nmeaSubscribe("GGA", myNmeaCallback);
  myGNSS.nmeaSubscribe("RMC", myNmeaCallback);
  myGNSS.nmeaSubscribe("PQTMPVT", myNmeaCallback);
  myGNSS.rtcmSubscribe(1005, myRtcmCallback);
  myGNSS.rtcmSubscribe(1124, myRtcmCallback);
  myGNSS.rtcmSubscribe(1074, myRtcmCallback);
}

void loop()
{
    Serial.println();
    Serial.println("Resetting device...");
    myGNSS.setModeRover();
    Serial.println("Here's the engine running in ROVER mode");
    monitorActivity(30);

    Serial.println();
    Serial.println("Resetting device...");
    myGNSS.setModeBase();
    Serial.println("Here's the engine running in BASE mode");
    monitorActivity(30);
}

void myNmeaCallback(NmeaPacket &packet)
{
  Serial.printf("$%s%s ", packet.TalkerId().c_str(), packet.SentenceId().c_str());
  if (packet.SentenceId() == "PQTMPVT")
    Serial.printf("%f ", myGNSS.getLatitude());
}

void myRtcmCallback(RtcmPacket &packet)
{
  Serial.printf("RTCM-%d ", packet.type);
}

void monitorActivity(int seconds)
{
  if (myGNSS.isConnected())
  {
    int ggaRate = 0, rmcRate = 0, svinstatusRate = 0, pvtRate = 0, plRate = 0, epeRate = 0, rtcm1005Rate = 0, rtcm107XRate = 0;
    int mode = 0;
    myGNSS.getMessageRate("GGA", ggaRate);
    myGNSS.getMessageRate("RMC", rmcRate);
    myGNSS.getMessageRate("PQTMSVINSTATUS", svinstatusRate, 1);
    myGNSS.getMessageRate("PQTMPVT", pvtRate, 1);
    myGNSS.getMessageRate("PQTMPL", plRate, 1);
    myGNSS.getMessageRate("PQTMEPE", epeRate, 2);
    myGNSS.getMessageRate("RTCM3-1005", rtcm1005Rate);
    myGNSS.getMessageRate("RTCM3-107X", rtcm107XRate);

    myGNSS.getMode(mode);
    Serial.printf("Mode reported as '%s'\r\n", mode == 1 ? "ROVER" : "BASE");
    Serial.printf("Msgs enabled: GGA=%c RMC=%c, SVINSTATUS=%c, PVT=%c, PL=%c EPE=%c 1005=%c 107X=%c\r\n",
      ggaRate ? 'Y' : 'N', rmcRate ? 'Y' : 'N', svinstatusRate ? 'Y' : 'N', pvtRate ? 'Y' : 'N',
      plRate ? 'Y' : 'N', epeRate ? 'Y' : 'N', rtcm1005Rate ? 'Y' : 'N', rtcm107XRate ? 'Y' : 'N');
  }

  for (unsigned long start = millis(); millis() - start < 1000 * seconds; )
    myGNSS.update();
  Serial.println();
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
