/*
  Enable HAS corrections
  By: Nathan Seidle and Mikal Hart
  Date: January 3, 2026
  License: Public domain - do whatever you'd like.

  This example shows how to enable and monitor the High Accuracy Service (HAS) convergence.

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
  Serial.println("SparkFun LG290P HAS E6 example");

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

  if (myGNSS.setPppSettings(2, 1) == true) // Enable HAS E6 corrections, use WGS84 datum
    Serial.println("HAS E6 corrections enabled.");
  else
    Serial.println("Failed to enable HAS E6 corrections.");

  delay(2000); // Wait before we read the settings

  int mode = 0;
  int datum = 0;
  int timeout = 0;
  float horstd = 0;
  float verstd = 0;

  // HAS E6 is conceptually the same as the B2b PPP service. Therefore we use the global 'PPP' moniker
  // in the same way we use 'GNSS' to encapsulate GPS with other constellations.

  if (myGNSS.getPppSettings(mode, datum, timeout, horstd, verstd) == true)
  {
    Serial.print("PPP Settings: mode=");
    Serial.print(mode);
    Serial.print(", datum=");
    Serial.print(datum);
    Serial.print(", timeout=");
    Serial.print(timeout);
    Serial.print(", horstd=");
    Serial.print(horstd);
    Serial.print(", verstd=");
    Serial.println(verstd);
  }
  else
  {
    Serial.println("Failed to get PPP settings info.");
  }
}

void loop()
{
  myGNSS.update(); // Must be called often to process incoming data

  if (myGNSS.getPppSolutionType() == 6 || myGNSS.getPppSolutionType() == 7)
  {
    if (myGNSS.getPppSolutionType() == 6)
      Serial.println("HAS Detected! PPP Solution: PPP Converging");
    else if (myGNSS.getPppSolutionType() == 7)
      Serial.println("HAS Converged! PPP Solution: PPP Converged");

    Serial.printf("DatumID: %d ", myGNSS.getDatumId());
    if (myGNSS.getDatumId() == 1)
      Serial.println("WGS84");
    else if (myGNSS.getDatumId() == 2)
      Serial.println("PPP Original");
    else if (myGNSS.getDatumId() == 3)
      Serial.println("CGCS2000");

    Serial.printf("Diff ID: %d ", myGNSS.getPppDifferentialId());
    if (myGNSS.getPppDifferentialId() == 9001)
      Serial.println("B2b PPP");
    else if (myGNSS.getPppDifferentialId() == 9002)
      Serial.println("HAS/E6");

    Serial.printf("Diff Age: %d seconds\r\n", myGNSS.getPppDifferentialAge());
  }
  else
  {
    Serial.printf("PPP Solution Type: %d ", myGNSS.getPppSolutionType());
    if (myGNSS.getPppSolutionType() == 1)
      Serial.println("Single");
    else if (myGNSS.getPppSolutionType() == 2)
      Serial.println("Differential (SBAS / DGPS)");
  }

  int horizontalPositionalAccuracy = myGNSS.get2DError() * 1000; // Convert meters to mm
  int threeDPositionalAccuracy = myGNSS.get3DError() * 1000;

  Serial.printf("2D Error: %dmm 3D Error: %dmm Sats Used: %d Sats in View: %d\r\n",
                horizontalPositionalAccuracy, threeDPositionalAccuracy, myGNSS.getSatellitesUsedCount(), myGNSS.getSatellitesInViewCount());

  Serial.println();

  // Generally speaking, delaying between checks is a bad idea because the LG290P is outputting so much serial.
  // Instead, call myGNSS.update() far more often in order to parse it all.
  // If you must do other things and delay a lot, then increase setRxBufferSize to 4096 before beginning the serial port (see above).
  delay(1000); // A bad delay to demonstrate the parsing issue
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
